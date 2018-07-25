# 当前运动状态，包括定位信息以及运动信息

Header header


# GPS 的信息，包括经纬度海拔
sensor_msgs/NavSatFix gps

# 地图坐标下的 6DOF 姿态，以及当前的三轴线速度角速度信息
nav_msgs/Odometry odom

# 包括朝向及三轴加速度和角速度
sensor_msgs/Imu imu
