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
     * check if the path or the node is passable.
     */
    bool passability_check( type_road_point );
    /*
     * yield ratation transform matrix(2x2), row-major order.
     */
    std::vector<double> yield_ratation_matrix( double source, double target );
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
    type_road_point yield_joint_by_distance(
        type_road_point begin, type_road_point end, double distance);
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
    /*
     * yield the expected velocity of selected path
     */
    void trim_tree();
    /*
     * selected select possible end nodes of potential paths.
     */
    std::vector<tree<type_node_point>::iterator> select_path_end_nodes();
    /*
     * selecte the end node with the minimum cost.
     */
    tree<type_node_point>::iterator select_path_end_node(
        std::vector<tree<type_node_point>::iterator> candidate_ends );
    /*
     * yield path base on the given path end node.
     */
    bool yield_selected_path( tree<type_node_point>::iterator path_end );
    /*
     * search parent of a node.
     */
    bool search_parent_node(type_node_point tnp);

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
       * delta_drain
       */
      tree<type_node_point>::iterator selected_path_end;
      /*
       * if distance between two nodes is smaller than this, 
       * they are considered as one nodes.
       */
      static constexpr double PERCISION = 1; 
      /*
       * length of local reference path
       */
      static constexpr double THRESHOLD = 40;
      /*
       * number of nodes in local reference path
       */
      static constexpr double JOINTS_NUMBER = 4;
      /*
       * the max longitudinal acceleration
       */
      static constexpr double LONGITUDINAL_ACC = 3;
      /*
       * the max lateral acceleration
       */
      static constexpr double LATERAL_ACC = 2;
      /*
       * the coefficient of deceleration process, velocity of end point == 0
       * duration of the whole deceleration process = sqrt( start_velocity / coefficient )
       */
      static constexpr double DEC_COEFF = 1;
      /*
       * 
       */
      static constexpr double VEHICLE_BOX_LENGTH = 5;
      /*
       * 
       */
      static constexpr double VEHICLE_BOX_WIDTH = 2.2;
      /*
       * 
       */
      static constexpr double VEHICLE_BOX_RESOLUTION = 0.18;
      /*
       * Waypoints density ( number/m), no including the end point of each segement.
       */
      static constexpr double WAYPOINTS_DENSITY = 0.3;
      /*
       * selected path
       */
      std::vector<type_road_point> selected_path;
};

}
#endif // FUNCTION_SIMPLIFIED_HPP
