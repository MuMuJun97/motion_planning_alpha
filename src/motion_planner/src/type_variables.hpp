#ifndef TYPE_VARIABLES_HPP
#define TYPE_VARIABLES_HPP
#include <eigen3/Eigen/Dense>
#include "tree.hh"
#include "tree_util.hh"
#include "Clothoid.h"
#include "dubins.h"
typedef struct type_road_point
{
    double x;
    double y;
    double latitude;
    double longitude;
    double angle;
}type_road_point;
typedef struct type_node_point
{
    double x;
    double y;
    double latitude;
    double longitude;
    double theta;
    double v;
    bool flag_effective;
    double cost;
    double k,dk,L;
}type_node_point;
typedef struct type_path_cost
{
    double dk;      //curvature change
    double length;  //length of the path
    double diff_curvature;  //difference of curvarture
    double index;   //index of the path
}type_path_cost;
typedef struct type_node
{
    bool flag_effective;
}type_node;
typedef struct vehicle_velocity
{
    double vx;
    double vy;
    double vz;
}vehicle_velocity;
#endif // TYPE_VARIABLES_HPP
