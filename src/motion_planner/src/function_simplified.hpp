#ifndef FUNCTION_SIMPLIFIED_HPP
#define FUNCTION_SIMPLIFIED_HPP
#include "type_variables.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <random>
#include <math.h>
#include "utility_func.hpp"
using namespace Clothoid;
using namespace Eigen;
using namespace utility_functions;
namespace func_simplified {
class fun_simple
{
public:
    fun_simple();
    ~fun_simple();
    /*
     * check the collision situation of the checked point.
     */
    bool collision_check(type_road_point point);
    /*
     * update the infomation include:
     * reference path,global coordinate, current speed.
     */
    void update_info(type_road_point global_coordinate);
    /*
     * add new node into the tree
     */
    bool add_node_into_tree(type_node_point new_node);
    /*
     * setup the planner
     * initialize the parameters used in the planner
     */
    void setup();
    /*
     * check whether the node is nearby the goal point
     */
    bool is_goal(type_node_point new_node);
    /*
     * initialize the tree
     */
    void initialize_tree();
    /*
     * search the best path
     */
    bool search_best_path();
    /*
     * re-propagating the updated vehicle state with the selected best path
     */
    bool repropagating();
    /*
     * yield local reference path from grobal path
     */
    void set_local_reference_path();
    /*
     * yield the node by distance between it and the vehicle location.
     */
    type_road_point yield_joints_by_distance(
        type_road_point begin, type_road_point end, double distance, double number);
    /*
     * yield the expected velocity of selected path
     */
    void yield_expected_speeds();
    /*
     * yield the expected velocity of each road point
     * mode = 0: cruise situation
     * mode = 1: norm slow down
     */
    double yield_expected_speed( type_road_point from, type_road_point here, int mode );

public:
    /*
      * set up the tree.
      */
      tree<type_node_point> m_tree;
      /*
       * global coordinate
       */
      type_road_point global_coord;
      /*
       * vehicle location
       */
      type_road_point vehicle_loc;
      /*
       * vehicle velocity
       */
      vehicle_velocity vehicle_vel;
      /*
       * global path from global planner
       */
      std::vector<type_road_point> global_path;
      /*
       * reference path
       */
      std::vector<type_road_point> local_reference_path;
      /*
       * grip map
       */
      GridMap local_grid_map;
      /*
       * current speed.
       */
      Speed speed;
      /*
       * goal nodes
       */
      type_road_point goal_point;
      /*
       * goal nodes before got the global goal point 
       */
      type_road_point local_goal;
      /*
       * save the best path
       */
      std::vector<type_road_point> best_path;
public://parameters used in the planner
      /*
       * delta_drain
       */
      double delta_drain;
      /*
       * goal region limitation
       */
      double goal_size;
      /*
       * weight coff for the selection of the parameters
       */
      double weight_dk,weight_diff_curvature,weight_length;
      /*
       * if distance between two nodes is smaller than this, 
       * they are considered as one nodes.
       */
      double PERCISION = 1; 
      /*
       * length of local reference path
       */
      double THRESHOLD = 40;
      /*
       * number of nodes in local reference path
       */
      double JOINTS_NUMBER = 3;
      /*
       * the max longitudinal acceleration
       */
      double LONGITUDINAL_ACC = 3;
      /*
       * the max lateral acceleration
       */
      double LATERAL_ACC = 2;
      /*
       * the coefficient of deceleration process, velocity of end point == 0
       * duration of the whole deceleration process = sqrt( start_velocity / coefficient )
       */
      double DEC_COEFF = 1;
      /*
       * selected path
       */
      std::vector<type_road_point> selected_path;


};

}
#endif // FUNCTION_SIMPLIFIED_HPP
