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

    std::normal_distribution<> normal_radius_dist( nRADIUS_0, nSIGMA_R );
    std::normal_distribution<> normal_theta_dist( nTHETA_0, nSIGMA_T );
    std::normal_distribution<> normal_heading_dist( nHEADING_0, nSIGMA_H );

    std::normal_distribution<> unsafe_radius_dist( uRADIUS_0, uSIGMA_R );
    std::normal_distribution<> unsafe_theta_dist( uTHETA_0, uSIGMA_T );
    std::normal_distribution<> unsafe_heading_dist( uHEADING_0, uSIGMA_H );
    std::uniform_int_distribution<> sign_dist(0, 1);

    int number = 0;
    int selected_index_in_reference_path;
    double r, theta, heading;

    std::vector<int> unsafe_joints_index;
    std::vector<int> normal_joints_index;

    for ( int i = 0; i < reference_path.size(); i++ )
    {
        if ( reference_path[i].state == 3 )
        {
            unsafe_joints_index.push_back( i );
        }
        else
        {
            normal_joints_index.push_back( i );
        }
    }

    std::uniform_int_distribution<> unsafe_joints_dist(
        0, unsafe_joints_index.size() - 1 );
    std::uniform_int_distribution<> normal_joints_dist(
        0, normal_joints_index.size() - 1 );

    std::uniform_real_distribution<> selection_dist(0,1);

    while (true) 
    {
        if ( unsafe_joints_index.size() < 1 )
        {
            int index = normal_joints_dist( mt );
            selected_index_in_reference_path = normal_joints_index[ index ];
            
            r = fabs( normal_radius_dist( mt ) );
            theta = normal_theta_dist( mt );
            heading = normal_heading_dist( mt );
        }
        else
        {
            if ( selection_dist(mt) > 0.4 )
            {
                int index = unsafe_joints_dist( mt );
                selected_index_in_reference_path = unsafe_joints_index[ index ];

                int sign = sign_dist( mt );
                if ( sign == 0 ) sign = -1;

                r = fabs( unsafe_radius_dist( mt ) );
                theta = sign * unsafe_theta_dist( mt );
                heading = unsafe_heading_dist( mt );
            }
            else
            {
                int index = normal_joints_dist( mt );
                selected_index_in_reference_path = normal_joints_index[ index ];
                
                r = fabs( normal_radius_dist( mt ) );
                theta = normal_theta_dist( mt );
                heading = normal_heading_dist( mt );
            }
        }

        number ++;

        type_road_point trp = reference_path[ selected_index_in_reference_path ];

        sample_node.x = trp.x + r * cos( trp.angle + theta );
        //printf( ">>> x: %f, r: %f, angle: %f, theta: %f", trp.x, r, trp.angle, theta);
        sample_node.y = trp.y + r * sin( trp.angle + theta);

        sample_node.angle = trp.angle + heading;

        if ( (*p_func_from_sampling) -> passability_check(sample_node) )
        {
            return true;
        }
        if( number > THRESHOLD )
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
