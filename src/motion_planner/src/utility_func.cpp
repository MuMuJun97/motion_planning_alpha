
#include "type_variables.hpp"

namespace utility_functions {
	type_road_point transform_from_node_to_point(type_node_point np)
	{
		type_road_point rp;
	    rp.x = np.x;
	    rp.y = np.y;
	    rp.angle = np.theta;
	    rp.latitude = np.latitude;
	    rp.longitude = np.longitude;
		rp.state = np.state;
		rp.size = true;
	    return rp;
	}
	double norm_sqrt(type_road_point rp1,type_road_point rp2)
	{
		//std::cout << "rp2: (" << rp2.x << ", " << rp2.y << ")" << std::endl;
		//std::cout << "rp1: (" << rp1.x << ", " << rp1.y << ")" << std::endl;
		return sqrt((rp2.x-rp1.x)*(rp2.x-rp1.x)+(rp2.y-rp1.y)*(rp2.y-rp1.y));
	}
	double cal_dubins_dis(type_node_point a, type_road_point b)
	{
	    DubinsPath dp;
	    double q0[]={a.x,a.y,a.theta};
	    double q1[]={b.x,b.y,b.angle};
	    dubins_init(q0,q1,5,&dp);
	    //std::cout << "a: " << a.x << " " << a.y << " " << a.theta << std::endl;
	    //std::cout << "b: " << b.x << " " << b.y << " " << b.angle << std::endl;
	   return dubins_path_length(&dp);
	}
}

