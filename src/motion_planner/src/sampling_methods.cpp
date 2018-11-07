#include "sampling_methods.h"
//using namespace utility_methods;
using namespace std;
sampling_methods::sampling_methods()
{
    R0=3;
    Sigma_R=1;
    Sigma_T=1;
    // p_func_from_sampling=new function_methods();
}
bool sampling_methods::sampling_nearby_reference_path(std::vector<type_road_point> reference_path)
{
    std::mt19937 mt(ran_device());
    std::normal_distribution<> norm_dist(0,1);
    std::uniform_real_distribution<> dist_real(-R0,R0);
    std::uniform_real_distribution<> goal_bias(0,1);
    std::uniform_real_distribution<> angle_dist(-M_PI/24,M_PI/24);
    int num_points_in_reference_path=(int)reference_path.size()-1;
    std::uniform_int_distribution<> uniform_dist(0,num_points_in_reference_path);
    int n=0;
    int selected_index_in_reference_path;
    while (true) {
        if (goal_bias(mt)>0.8)
            selected_index_in_reference_path = uniform_dist(mt);
        else
            selected_index_in_reference_path = 0;
        //R0=(*p_func_from_sampling)->sampling_parameters[selected_index_in_reference_path].first;
        n++;
        double ra=norm_dist(mt);

        double R=R0+Sigma_R*fabs(ra);
        //  double theta=reference_path[selected_index_in_reference_path].angle+Sigma_T*ra;
        sample_node.x = reference_path[selected_index_in_reference_path].x + dist_real(mt);
        sample_node.y = reference_path[selected_index_in_reference_path].y + dist_real(mt);//+R*sin(theta);
        //sample_node.angle = M_PI/2+angle_dist(mt);
        sample_node.angle = reference_path[selected_index_in_reference_path].angle + angle_dist(mt);
        if ( (*p_func_from_sampling) -> passability_check(sample_node) )
        {
            return true;
        }
        if(n>10)
        {
            cout<<"can not find a feasible sample"<<endl;
            return false;
        }
    }
}
void sampling_methods::tune_sampling_parameters(double dist,double D_dist)
{
    R0=0.5;
}
