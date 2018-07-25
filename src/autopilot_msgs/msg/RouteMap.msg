Header header

# 地图原点的经纬度信息
RouteNode origin

# 路网图是一个有向有环且全连通的稀疏图，用邻接矩阵标识空间复杂度太大，暂时拟定用边集表示。在做规划的时候可以自行还原成邻接矩阵形式。

# 图中的边集
RouteEdge[] edges

# 特定标示区域
RouteArea[] areas