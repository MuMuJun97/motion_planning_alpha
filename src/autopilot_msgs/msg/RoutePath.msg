# autopilot_msgs/RoutePath.msg
Header header

# 需要途经的目标位置序列
RouteNode[] goals

# 目标位置对应的车身朝向，考虑控制器的限制，暂时可以忽略
# 朝向角相对于地图坐标系的 0 度，地理坐标系的正北，单位 rad
float64[] headings