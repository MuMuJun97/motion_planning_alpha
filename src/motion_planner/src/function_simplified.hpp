#ifndef FUNCTION_SIMPLIFIED_HPP
#define FUNCTION_SIMPLIFIED_HPP
#include "type_variables.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <random>
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
    void update_info(type_road_point global_coordinate, std::vector<type_road_point> reference_path,double current_speed);
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
       * reference path
       */
      std::vector<type_road_point> local_reference_path;
      /*
       * current speed.
       */
      double speed;
      /*
       * goal nodes
       */
      type_road_point goal_point;
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
       * selected path
       */
      std::vector<type_road_point> selected_path;


};

}
#endif // FUNCTION_SIMPLIFIED_HPP
