"""
Rosbridge class:

Class that handle communication between CARLA and ROS
"""
import random
import rospy
from itertools import count
import tf
import numpy as np
import libGaussLocalGeographicCS as Map2Wgs

from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

from carla.settings import CarlaSettings
from carla_ros_bridge.control import InputController
from carla_ros_bridge.markers import PlayerAgentHandler, NonPlayerAgentsHandler
from carla_ros_bridge.sensors import CameraHandler, LidarHandler
from carla_ros_bridge.map import MapHandler

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float64
from autopilot_msgs.msg import *

class CarlaRosBridge(object):
    """
    Carla Ros bridge
    """

    def __init__(self, client, params):
        """

        :param params: dict of parameters, see settings.yaml
        :param rate: rate to query data from carla in Hz
        """
        self.setup_carla_client(client=client, params=params)
        self.frames_per_episode = params['Framesperepisode']

        self.tf_to_publish = []
        self.msgs_to_publish = []
        self.publishers = {}

        # definitions useful for time
        self.cur_time = rospy.Time.from_sec(
            0)  # at the beginning of simulation
        self.carla_game_stamp = 0
        self.carla_platform_stamp = 0

        # creating handler to handle vehicles messages
        self.player_handler = PlayerAgentHandler(
            "player_vehicle", process_msg_fun=self.process_msg)
        self.non_players_handler = NonPlayerAgentsHandler(
            "vehicles", process_msg_fun=self.process_msg)

        # creating handler for sensors
        self.sensors = {}
        for name, _ in self.param_sensors.items():
            self.add_sensor(name)

        # creating input controller listener
        self.input_controller = InputController()

        # record msg from controller
        float64multiarray = Float64MultiArray()
        float64multiarray.data.append(0.0)
        float64multiarray.data.append(0.0)
        float64 = Float64()
        float64.data = 0.0
        self.controller_msg = {
            'steering' : float64multiarray,
            'throttle' : float64,
            'brake' : float64
        }
        self.player_odometry = Odometry()
        self.sub_steer = (
            rospy.Subscriber(
                'player_steering', Float64MultiArray, self.get_steering_msg)
        )
        self.sub_throttle = (
            rospy.Subscriber('player_throttle', Float64, self.get_throttle_msg)
        )
        self.sub_brake = (
            rospy.Subscriber('player_brake', Float64, self.get_brake_msg)
        )



    def setup_carla_client(self, client, params):
        self.client = client
        self.param_sensors = params.get('sensors', {})
        self.carla_settings = CarlaSettings()
        self.carla_settings.set(
            SendNonPlayerAgentsInfo=params.get('SendNonPlayerAgentsInfo', True),
            NumberOfVehicles=params.get('NumberOfVehicles', 20),
            NumberOfPedestrians=params.get('NumberOfPedestrians', 40),
            WeatherId=params.get('WeatherId', random.choice([1, 3, 7, 8, 14])),
            SynchronousMode=params.get('SynchronousMode', True),
            QualityLevel=params.get('QualityLevel', 'Low')
            )
        self.carla_settings.randomize_seeds()

    def add_sensor(self, name):
        rospy.loginfo("Adding sensor {}".format(name))
        sensor_type = self.param_sensors[name]['SensorType']
        params = self.param_sensors[name]['carla_settings']
        sensor_handler = None
        if sensor_type == 'LIDAR_RAY_CAST':
            sensor_handler = LidarHandler
        if sensor_type == 'CAMERA':
            sensor_handler = CameraHandler
        if sensor_handler:
            self.sensors[name] = sensor_handler(
                name,
                params,
                carla_settings=self.carla_settings,
                process_msg_fun=self.process_msg)
        else:
            rospy.logerr(
                "Unable to handle sensor {name} of type {sensor_type}".format(
                    sensor_type=sensor_type, name=name))

    def on_shutdown(self):
        rospy.loginfo("Shutdown requested")

    def process_msg(self, topic=None, msg=None):
        """
        Function used to process message

        Here we create publisher if not yet created
        Store the message in a list (waiting for their publication) with their associated publisher

        Messages for /tf topics are handle differently in order to publish all transform in the same message
        :param topic: topic to publish the message on
        :param msg: ros message
        """
        if topic not in self.publishers:
            if topic == 'tf':
                self.publishers[topic] = rospy.Publisher(
                    topic, TFMessage, queue_size=100)
            else:
                self.publishers[topic] = rospy.Publisher(
                    topic, type(msg), queue_size=10)

        if topic == 'tf':
            # transform are merged in same message
            self.tf_to_publish.append(msg)
        else:
            self.msgs_to_publish.append((self.publishers[topic], msg))

    def send_msgs(self):
        for publisher, msg in self.msgs_to_publish:
            publisher.publish(msg)
        self.msgs_to_publish = []

        tf_msg = TFMessage(self.tf_to_publish)
        self.publishers['tf'].publish(tf_msg)
        self.tf_to_publish = []

    def compute_cur_time_msg(self):
        self.process_msg('clock', Clock(self.cur_time))

    def run(self):
        self.publishers['clock'] = rospy.Publisher(
            "clock", Clock, queue_size=10)

        # load settings into the server
        scene = self.client.load_settings(self.carla_settings)

        # Choose one player start at random.
        number_of_player_starts = len(scene.player_start_spots)
        player_start = random.randint(0, max(0, number_of_player_starts - 1))
        
        player_start = 10

        # Send occupancy grid to rivz
        map_handler = MapHandler(scene.map_name)
        map_handler.send_map()

        self.client.start_episode(player_start)

        # publish start spots
        self.generate_start_spots_msg(scene)

        # record player's rotation of last frame [roll, pitch, yaw, game_timestamp]
        last_rotation = [0 , 0, 0, 0]

        # record player's velocity of last frame
        last_velocity = [0, 0, 0, 0]

        for frame in count():
            if (frame == self.frames_per_episode) or rospy.is_shutdown():
                break
            measurements, sensor_data = self.client.read_data()

            # handle time
            self.carla_game_stamp = measurements.game_timestamp
            self.cur_time = rospy.Time.from_sec(self.carla_game_stamp * 1e-3)
            self.compute_cur_time_msg()

            # handle agents
            self.player_handler.process_msg(
                measurements.player_measurements, cur_time=self.cur_time)
            self.non_players_handler.process_msg(
                measurements.non_player_agents, cur_time=self.cur_time)

            # handle sensors
            for name, data in sensor_data.items():
                self.sensors[name].process_sensor_data(data, self.cur_time)

            # calculate player's velocity of this frame
            current_velocity = self.calculate_current_velocity(
                measurements, last_velocity )

            # generate generate_player_odometry_msg
            self.generate_player_odometry_msg(measurements, current_velocity)

            # generate generate_motion_state_msg
            self.generate_motion_state_msg(measurements, last_rotation)

            # publish all messages
            self.send_msgs()

            # handle control
            if rospy.get_param('carla_autopilot', True):
                control = measurements.player_measurements.autopilot_control
                self.client.send_control(control)
            else:
                # control = self.input_controller.cur_control
                control = self.generate_controller(measurements)
                self.client.send_control(control)

            # update player's rotation of this frame
            last_rotation = self.update_last_rotation(measurements)

            # record player's velocity of this frame
            last_velocity = current_velocity
            



    #####[Fllowing codes added by Trouble,is not the first-party codes]#####
    def generate_start_spots_msg(self, scene):
        """
        Publish the player_start_spots to ros
        """
        _player_start_spots = Float64MultiArray()
        _dim_0 = MultiArrayDimension()
        _dim_0.label = 'height'
        _dim_0.size = len(scene.player_start_spots)
        _player_start_spots.layout.dim.append(_dim_0)
        _dim_1 = MultiArrayDimension()
        _dim_1.label = 'width'
        _dim_1.size = 3
        _player_start_spots.layout.dim.append(_dim_1)
        for spot in scene.player_start_spots:
            _player_start_spots.data.append(spot.location.x)
            _player_start_spots.data.append(spot.location.y)
            _player_start_spots.data.append(spot.location.z)
        self.process_msg('player_start_spots', _player_start_spots)

    def generate_player_odometry_msg(self, measurements, current_velocity):
        rotation = measurements.player_measurements.transform.rotation
        location = measurements.player_measurements.transform.location

        quat = tf.transformations.quaternion_from_euler(
            np.radians(rotation.roll),
            np.radians(rotation.pitch),
            np.radians(rotation.yaw)
        )

        odometry = Odometry()
        odometry.pose.pose.position.x = location.x
        odometry.pose.pose.position.y = location.y
        odometry.pose.pose.position.z = location.z
        odometry.pose.pose.orientation.x = quat[0]
        odometry.pose.pose.orientation.y = quat[1]
        odometry.pose.pose.orientation.z = quat[2]
        odometry.pose.pose.orientation.w = quat[3]
        
        odometry.twist.twist.linear.x = current_velocity[0]
        odometry.twist.twist.linear.y = current_velocity[1]
        odometry.twist.twist.linear.z = current_velocity[2]

        self.player_odometry = odometry
        self.process_msg("player_odometry", odometry)
    
    def generate_motion_state_msg(self, measurements, last_rotation):
        motion_state = MotionState()
        # generate IMU msg
        acceleration = measurements.player_measurements.acceleration
        rotation = measurements.player_measurements.transform.rotation
        quat = tf.transformations.quaternion_from_euler(
            np.radians(rotation.roll),
            np.radians(rotation.pitch),
            np.radians(rotation.yaw - 90)
        )
        imu = Imu()
        imu.orientation.x = quat[0]
        imu.orientation.y = quat[1]
        imu.orientation.z = quat[2]
        imu.orientation.w = quat[3]

        imu.angular_velocity.x = (
            np.radians( rotation.roll - last_rotation[0] ) /
            ( measurements.game_timestamp - last_rotation[3] ) * 1e3
        )
        imu.angular_velocity.y = (
            np.radians( rotation.pitch - last_rotation[1] ) /
            ( measurements.game_timestamp - last_rotation[3] ) * 1e3
        )
        imu.angular_velocity.z = (
            np.radians( rotation.yaw - last_rotation[2] ) /
            ( measurements.game_timestamp - last_rotation[3] ) * 1e3
        )

        imu.linear_acceleration.x = acceleration.x
        imu.linear_acceleration.y = acceleration.y
        imu.linear_acceleration.z = acceleration.z

        # generate odometry msg
        odometry = Odometry()
        odometry = self.player_odometry

        # generate GPS msg
        gps = NavSatFix()
        map2wsg = Map2Wgs.GaussLocalGeographicCS(22.9886565512, 113.2691559583)

        llh = map2wsg.xyz2llh(
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            0
        )
        gps.latitude = llh[0]
        gps.longitude = llh[1]
        gps.altitude = llh[2]

        motion_state.imu = imu
        motion_state.odom = odometry
        motion_state.gps = gps

        self.process_msg("/localization/motion_state", motion_state)

    def calculate_current_velocity(self, measurements, last_velocity):
        current_velocity = [0, 0, 0, 0]
        current_velocity[0] = (
                last_velocity[0] + 
                measurements.player_measurements.acceleration.x * 
                (measurements.game_timestamp - last_velocity[3]) * 1e-3
            )
        current_velocity[1] = (
                last_velocity[1] + 
                measurements.player_measurements.acceleration.y * 
                (measurements.game_timestamp - last_velocity[3]) * 1e-3
            )
        current_velocity[2] = (
                last_velocity[2] + 
                measurements.player_measurements.acceleration.z * 
                (measurements.game_timestamp - last_velocity[3]) * 1e-3
            )
        current_velocity[3] = measurements.game_timestamp
        return current_velocity

    def update_last_rotation(self, measurements):
        updated_rotation = [0, 0, 0, 0]
        updated_rotation[0] = (
            measurements.player_measurements.transform.rotation.roll
        )
        updated_rotation[1] = (
                measurements.player_measurements.transform.rotation.pitch
        )
        updated_rotation[2] = (
                measurements.player_measurements.transform.rotation.yaw
        )
        updated_rotation[3] = measurements.game_timestamp
        return updated_rotation

    def generate_controller(self, measurements):
        control = measurements.player_measurements.autopilot_control
        control.steer = - self.controller_msg['steering'].data[0] / 70
        control.throttle = self.controller_msg['throttle'].data
        control.brake = self.controller_msg['brake'].data
        control.reverse = False
        control.hand_brake = False
        rospy.loginfo(control)
        return control

    def get_steering_msg(self, Float64MultiArray):
        rospy.loginfo('Starting get steering msg from controller')
        self.controller_msg['steering'] = Float64MultiArray
    def get_throttle_msg(self, Float64):
        rospy.loginfo('Starting get steering msg from controller')
        self.controller_msg['throttle'] = Float64
    def get_brake_msg(self, Float64):
        rospy.loginfo('Starting get steering msg from controller')
        self.controller_msg['brake'] = Float64

    ######[Above codes added by Trouble,is not the first-party codes]###########

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        rospy.loginfo("Exiting Bridge")
        return None
