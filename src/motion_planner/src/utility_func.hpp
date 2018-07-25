#ifndef UTILITY_FUNC_HPP
#define UTILITY_FUNC_HPP
#include "type_variables.hpp"
#include "dubins.h"
namespace utility_functions {
	type_road_point transform_from_node_to_point(type_node_point np);
	double norm_sqrt(type_road_point rp1,type_road_point rp2);
	//calculate a dubins distance between a node and a road point.
	double cal_dubins_dis(type_node_point a,type_road_point b);
}
#endif // UTILITY_FUNC_HPP
