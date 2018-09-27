# Codes Related
## Bulid and set environment
0. Install OS. Recommanded OS: Ubuntu 16.04 LTS. it is tested.
1. Install ROS. Recommanded ROS version : [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu).
2. Install [gird map](https://github.com/anybotics/grid_map).
3. Install [google glog](https://github.com/google/glog).
 <br>3.1. ```$ cd```
 <br>3.2. ```$ git clone https://github.com/google/glog.git```
 <br>3.3. ```$ cd glog```
 <br>3.4. ```$ ./autogen.sh && ./configure && make && sudo make install```
1. Copy the whole file folder named *motion_planning_alpha* to the *<path>* you want it stays.
2. In your terminal, run command ```(~)$ cd <path>/motion_planning_alpha/```
3. In your terminal, run command ```(motion_planning_alpha)$ catkin_make```
 <br>3.1. if it fails, do more 5 times tries.
 <br>3.2. if the above method also fail, google why.
4. In your terminal, run command ```(motion_planning_alpha)$ echo "source <path>/motion_planning_alpha/devel/setup.bash" >> ~/.bashrc```

## Run
1. To start ROS services ```(motion_planning_alpha)$ roscore```
2. 每次更改代码之后都要: ```(motion_planning_alpha)$ catkin_make``` and ```(motion_planning_alpha)$ source devel/setup.bash```
<br>2.1 every time you create new msg or srv, run ```(motion_planning_alpha)$ catkin_make install```
<br>2.2 if you create a new code file, run ```(<path>)$ chmod +x <your_new_code_file>``` then  run ```(motion_planning_alpha)$ catkin_make``` 
3. 开启motion planner: ```(motion_planning_alpha)$ rosrun motion_planner motion_planner```
4. 开启global planner ```(motion_planning_alpha)$ rosrun global_planner planner.py```
5. 发送一次route map信息: ```(motion_planning_alpha)$ rosbag play routemap.bag```
6. 发送位置信息: ```(motion_planning_alpha)$ rostopic pub /localization/location autopilot_msgs/Location  '{stamp: now, frame_id: map}' '11290.4667969' '8706.98730469' '[0.0, 0.0, 0.0, 0.0]' '23.067371' '113.3795204' '[0.0, 0.0, 0.0, 0.0]'```
7. 发送目标信息: ```(motion_planning_alpha)$ rostopic pub /route/goal autopilot_msgs/RoutePath  '{stamp: now, frame_id: map}' '[{latitude: 23.0677242, longitude: 113.3795769, x: 11296.2177734, y: 8746.04199219}]' '[0.0]'```
8. Publish the grid map
 <br>```$ roslaunch segmentation.launch```
 <br>```$ rosbag paly <your_3d_map>```
 <br>```$ rviz #to visualization for debug```

# Theory Related
## Overview

<div align=center> 
<img src="https://github.com/TroubleLi/Motion_Planning_alpha/blob/master/docs/graph.svg" 
width="500" alt="IMG LOST" title="IN GOD WE TRUST"/>
</div>

## 坐标变换树

<div align=center> 
<img src="https://github.com/TroubleLi/Motion_Planning_alpha/blob/master/docs/tf_tree.svg" 
width="500" alt="IMG LOST" title="IN GOD WE TRUST"/>
</div>

## 传感器数据

传感器数据由驱动发布，一般只使用内置类型

 - **激光点云**：`/velodyne_points`，[sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
 - **IMU**: `/imu/data`，[sensor_msgs/Imu.msg](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
 - **GPS**：`/gps/fix`，[sensor_msgs/NavSatFix.msg](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)

---

## 地图

### 2D 路沿栅格地图

包含道路上路沿等静态障碍物，作为辅助定位和规划避障的输入。

**话题名：**`/map/grid_map`，格式为 [nav_msgs/OccupancyGrid.msg](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)

```
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
```

### 道路网络图

路网由地图中所有可行驶的路线和特定标识区域（如红绿灯）组成，实质上是由一个有向有环图和表示区域的多边形组成。

路线网络图是行车路由的依据；标识区域是规划动作及策略的触发检测条件，如在红绿灯路口检测红绿灯，在行人区域检测动态障碍物等。

**话题名：**`/map/route_map`,

**数据格式：**
autopilot_msgs/RouteMap.msg

```
Header header

# 地图原点的经纬度信息
RouteNode origin

# 路网图是一个有向有环且全连通的稀疏图，用邻接矩阵标识空间复杂度太大，暂时拟定用边集表示。在做规划的时候可以自行还原成邻接矩阵形式。

# 图中的边集
RouteEdge[] edges

# 特定标示区域
RouteArea[] areas
```

边的数据格式：
autopilot_msgs/RouteEdge.msg

```
# 边的起点
RouteNode begin
# 边的终点
RouteNode end

# 权重，暂时没用到
float32 weight
```

区域的数据结构：
autopilot_msgs/RouteArea.msg

```
UInt16 id

# 类型
string type

# 由一系列节点组成，首尾相连组成一个区域
RouteNode[] nodes
```

图（Graph）的节点（Node）表示地图上的坐标：
autopilot_msgs/RouteNode.msg

```
# 纬度
float64 latitude
# 经度
float64 longitude

# 地图坐标，单位 m
float32 x
float32 y
```

### 坐标转换 service

提供经纬度和地图坐标的转换，服务名：`/map/wgs_to_map`。

autopilot_msgs/WGS2Map.srv

```
# 纬度
float64 latitude
# 经度
float64 longitude
---
# 地图坐标，单位 m
float32 x
float32 y
```

服务名：`/map/map_to_wgs`。

autopilot_msgs/Map2WGS.srv

```
# 地图坐标，单位 m
float32 x
float32 y
---
# 纬度
float64 latitude
# 经度
float64 longitude
```

---

## 融合定位

定位模块接收各个传感器数据及 2D 环境地图，输出高可信度的定位。

发布的**话题名**拟定为 `/localization/location`，输出格式：

autopilot_msgs/Location.msg

```
Header header

# 当前位置

# 地图坐标，单位 m
float32 x
float32 y
# 协方差
float64[4] map_covariance

# 纬度
float64 latitude
# 经度
float64 longitude
# 协方差
float64[4] wgs_covariance
```

为兼容 ROS 原有的可视化，同时输出 [nav_msgs/Odometry.msg](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) 类型的局部地图坐标系里程计信息，**话题名**为 `/localization/map_location`，

并发布 `map` 到 `vehicle_footprint` 的 TF 坐标变换链接。

---

此外，定位模块还负责计算输出当前车体的运动状态，**话题名** 为 `/localization/motion_state` ，类型为 `autopilot_msgs/MotionState.msg`

```
# 当前运动状态，包括定位信息以及运动信息

Header header


# GPS 的信息，包括经纬度海拔
sensor_msgs/NavSatFix gps

# 地图坐标下的 6DOF 姿态，以及当前的三轴线速度角速度信息。局部地图坐标系原点指向正北。
nav_msgs/Odometry odom

# 包括朝向及三轴加速度和角速度，朝向以正北方向为原点。
sensor_msgs/Imu imu
```

---

## 全局规划

全局规划根据定位和目标信息，基于路线网络图进行路由，输出行车路线。

**输入：** 

1. `/map/route_map` 话题的路网图，autopilot_msgs/RouteMap.msg（见上文）
2. `/localization/location` 话题的定位信息，autopilot_msgs/Location.msg
3. `/route/goal` 话题的目标信息，包含途经点和终点，autopilot_msgs/RoutePath.msg

```
# autopilot_msgs/RoutePath.msg
Header header

# 需要途经的目标位置序列
RouteNode[] goals

# 目标位置对应的车身朝向，考虑控制器的限制，暂时可以忽略
# 朝向角相对于地图坐标系的 0 度，地理坐标系的正北，单位 rad
float64[] headings
```

**输出：**

`/route/path` 话题名的行车路线，格式为 autopilot_msgs/RoutePath.msg，同上。

---

## 障碍物检测

障碍物检测模块输入激光雷达和毫米波雷达等传感器数据，输出静态障碍物和动态障碍物信息。
### 静态障碍物

输出**车身坐标系 (vehicle_footprint) 下**的占用栅格图：

**话题名：** `/detection/static_obstacle_grid`，数据格式：[nav_msgs/OccupancyGrid.msg](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)

### 动态障碍物

输出**车身坐标系 (vehicle_footprint) 下**的毫米波障碍物追踪信息：

**话题名：**`/detection/dynamic_obstacle`，数据格式：[delphi_esr_msgs/EsrTrack.msg](https://github.com/astuff/astuff_sensor_msgs/blob/3db343743157f475f8f9f8f87035a71372db59dc/delphi_esr_msgs/msg/EsrTrack.msg)
```
Header header

# ESR Track Msg
string        canmsg
uint8         track_ID
float32       track_lat_rate
bool          track_group_changed
uint8         track_status
float32       track_angle
float32       track_range
bool          track_bridge_object
bool          track_rolling_count
float32       track_width
float32       track_range_accel
uint8         track_med_range_mode
float32       track_range_rate
```

---

## 局部规划

局部规划根据全局路线，向下对接控制器，输出控制轨迹和速度，
同时根据动静态障碍物信息，局部实时避障；根据地图标示区域，做指定动作，或选择不同策略。

**输入：**

1. 可信定位信息：`/localization/location`   (useless)
2. 路由输出的全局路径：`/route/path`
3. 地图：`/map/grid_map`，`/map/route_map`
4. 障碍物：`/detection/static_obstacle_grid`，`/detection/dynamic_obstacle`
5. 车体运动状态： `/localization/motion_state`  //jingweidu ?

**输出：**

输出轨迹航点 waypoints，话题名为：`/planner/way_points` ，类型为 `autopilot_msgs/WayPoints.msg`。

```
# autopilot_msgs/WayPoints.msg
Header header

# 途经的航点
# 以点的经纬度为准，需要明确赋值
RouteNode[] points

# 每个点对应的期望速度，单位 m/s
float64[] speeds   //fangxiang
```
