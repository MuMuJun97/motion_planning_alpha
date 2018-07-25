#include <utility_func.hpp>
namespace utility_functions {
void transform_from_node_to_point(type_node_point np, type_road_point rp)
{
    rp.x=np.x;
    rp.y=np.y;
    rp.angle=np.theta;
}
double norm_sqrt(type_road_point rp1,type_road_point rp2)
{
    return sqrt((rp2.x-rp1.x)*(rp2.x-rp1.x)+(rp2.y-rp1.y)*(rp2.y-rp1.y));
}
}
