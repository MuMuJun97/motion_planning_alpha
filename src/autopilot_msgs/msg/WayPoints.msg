# autopilot_msgs/WayPoints.msg
Header header

# 途经的航点
# 以点的经纬度为准，需要明确赋值
RouteNode[] points

# 每个点对应的期望速度，单位 m/s
float64[] speeds