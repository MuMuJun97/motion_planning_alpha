# autopilot_msgs

东风 AX7 自动驾驶项目中的自定义 ROS 消息数据格式，具体接口定义与介绍见[文档](docs/各模块输出及数据格式定义.md)。

## 编译

如果用到该 package 的 Message 类型，编译和链接时需要指明该项依赖，包括 `package.xml` 和 `CMakeList.txt` 两处。

## package.xml

加入:

```xml
<build_depend>autopilot_msgs</build_depend>
<exec_depend>autopilot_msgs</exec_depend>
```

如果 `package.xml` 是旧版本格式，`exec_depend` 改为 `run_depend` 。

### CMakeList.txt

1. 在 `find_package(catkin REQUIRED COMPONENTS` 后加入依赖，如：

```
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        autopilot_msgs
        )
```

2. 在 `catkin_package()` 其中的 `CATKIN_DEPENDS` 后面加入依赖：如：

```
catkin_package(
        CATKIN_DEPENDS roscpp rospy autopilot_msgs
)
```

3. 给用到 autopilot_msgs 消息的 Target 指明依赖关系：

```
add_dependencies(your_target autopilot_msgs_generate_messages_cpp)
```

## 使用

与一般 Message 无异，参照 ROS 官方文档。
