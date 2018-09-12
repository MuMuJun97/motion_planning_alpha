# LCM整理
## 控制层相关LCM整理
### 1. 惯导位置信息
**惯导信息**
```c
package obu_lcm;

struct ins_info
{
    double gps_time;                //gps时间
    int32_t week;                   //星期，周日=0，周一=1，……，周六=6
    double lat;                     //经度
    double lon;                     //纬度
    double height;                  //高程
    double lateral_speed;           //横向速度，单位：m/s
    double longitudinal_speed;      //纵向速度，单位：m/s
    double down_speed;              //地向速度，单位：m/s
    double roll;                    //横滚角度，单位：度
    double pitch;                   //俯仰角度，单位：度
    double heading;                 //航向角度，单位：度
    double lateral_accelerate;      //横向加速度，单位：m/s2
    double longitudinal_accelerate; //纵向加速度，单位：m/s2
    double down_accelerate;         //地向加速度，单位：m/s2
    double roll_speed;              //横滚角速度，单位：度/s
    double pitch_speed;             //俯仰角速度，单位：度/s
    double heading_speed;           //航向角速度，单位：度/s
    int32_t flag;                   //定位定姿状态，源自POS状态量的高2位，参考枚举值：GPS_FLAG_XXX
    int32_t n;                      //RTK状态，源自POS状态量的低6位，参考枚举值：GPS_N_XXX
}
```

**说明**

- 无速度编码器，车辆速度也必须在ins_info里取得
- 由imu_parser发出，需要位置信息的都可以接入
- 发布频率为稳定的100Hz

### 2.  回馈信息
**转向回馈**
```c
struct steering_feedback_info
{
    double steering_angle;       //方向盘转角，单位：度
    double steering_angle_speed; //方向盘角速度，单位：度/s
}
```

**说明**

- 油门、刹车无回馈信息
- 不以转向回馈做闭环，仅用来显示车辆状态

### 3.  底层执行器接口
**转向控制**
```c
struct steering_control_info
{
    double steering_angle;       //方向盘转角，单位：度
    double steering_angle_speed; //方向盘角速度，单位：度/s
}
```
**油门控制**
```c
struct accelerate_control_info
{
   double high_sensor_v; //油门踏板传感器1比例电压，单位：v
   double low_sensor_v;  //油门踏板传感器2比例电压，单位：v
}
```
**刹车控制**
```c
struct brake_control_info {
    double brake_percentage;  //刹车百分比，0%表示不刹车，100%表示刹死
}
```

### 4. 规划层-控制层接口
**导航控制点定义**
```c
struct nav_points
{
    double p_x;                      //全局平面坐标
    double p_y;                      //全局平面坐标
    double speed_desired_Uxs;        //期望纵向速度，单位：m/s
}
```
**导航精确控制点的序列**
``` c
struct nav_control_points
{
    int64_t utime;                    //规划时间
    int32_t num_of_points;            //导航控制点数量，自身→前方
    nav_points points[num_of_points]; //导航控制点序列
}
```
**说明**

- nav_control_points包含一系列nav_points，规划层通过这个lcm将规划好的路径发布给控制层
- nav_points中p_x，p_y是全局平面坐标，由经纬度经过墨卡拖投影获得，原点为投影时定义的原点，由环境变量WORKPATH定义的目录下的local.txt定义，投影的方法见“LocalGeographicCS.hpp”；speed_desired_Uxs指到达这一个点时期望的纵向速度
- nav_control_points中的规划时间暂时没有用到。

## 规划层LCM
### 1. 规划层-控制层接口
同上，略

### 2. 惯导位置信息
同上，略

### 3. 雷达-规划层接口
**障碍物点定义**
```c
struct nad_obstacle
{
    int16_t    type;
    std::string source;
    double     lon;
    double     lat;
    double     x;
    double     y;
    double     width;
    double     height;
    double     yaw;
    double     speed;
    double     gps_time;
}
```
**障碍物信息**
```c
struct obstacle_info
{
    int16_t    num_of_obstacle;
    nad_obstacle obu_obstacle[num_of_obstacle]; 
}
```

**说明**

- obstacle_info包含一系列nad_obstacle，雷达通过这个lcm将规划好的路径发布给控制层
- nad_obstacle中的坐标系的定义为：以车辆重心为原点（重心假设为车辆中央扶手箱），x轴方向为车辆正右方，y轴方向为车辆正前方，z轴方向为车辆正上方。
