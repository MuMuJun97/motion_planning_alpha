"""
Rosbridge class:

Class that handle communication between CARLA and ROS
"""
import random
import rospy
from itertools import count
import tf
import numpy as np

from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

from carla.settings import CarlaSettings
from carla_ros_bridge.control import InputController
from carla_ros_bridge.markers import PlayerAgentHandler, NonPlayerAgentsHandler
from carla_ros_bridge.sensors import CameraHandler, LidarHandler
from carla_ros_bridge.map import MapHandler

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


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

        # Send occupancy grid to rivz
        map_handler = MapHandler(scene.map_name)
        map_handler.send_map()

        self.client.start_episode(player_start)

        # publish start spots
        self.publish_start_spots(scene)

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

            # publish motion state
            self.publish_motion_state(measurements)

            # publish all messages
            self.send_msgs()

            # handle control
            if rospy.get_param('carla_autopilot', True):
                control = measurements.player_measurements.autopilot_control
                self.client.send_control(control)
            else:
                control = self.input_controller.cur_control
                self.client.send_control(**control)

    #####[Fllowing codes added by Trouble,is not the first-party codes]#####
    def publish_start_spots(self, scene):  
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
        rospy.loginfo(_player_start_spots.layout.dim[0].size)
        for spot in scene.player_start_spots:
            _player_start_spots.data.append(spot.location.x)
            _player_start_spots.data.append(spot.location.y)
            _player_start_spots.data.append(spot.location.z)
        self.process_msg('player_start_spots', _player_start_spots)

    def publish_motion_state(self, measurements):
        forward_speed = measurements.player_measurements.forward_speed
        rotation = measurements.player_measurements.transform.rotation
        location = measurements.player_measurements.transform.location

        quat = tf.transformations.quaternion_from_euler(
            np.radians(rotation.roll),
            np.radians(rotation.pitch),
            np.radians(rotation.yaw)
        )
        velocity_x = forward_speed * np.cos( np.radians(rotation.yaw) )
        velocity_y = forward_speed * np.sin( np.radians(rotation.yaw) )

        odometry = Odometry()
        odometry.pose.pose.position.x = location.x
        odometry.pose.pose.position.y = location.y
        odometry.pose.pose.position.z = location.z
        odometry.pose.pose.orientation.x = quat[0]
        odometry.pose.pose.orientation.y = quat[1]
        odometry.pose.pose.orientation.z = quat[2]
        odometry.pose.pose.orientation.w = quat[3]
        
        odometry.twist.twist.linear.x = velocity_x
        odometry.twist.twist.linear.y = velocity_y
        odometry.twist.twist.linear.z = 0
        self.process_msg("player_motion_state", odometry)

    ######[Above codes added by Trouble,is not the first-party codes]###########

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        rospy.loginfo("Exiting Bridge")
        return None
