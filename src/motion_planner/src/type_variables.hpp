#ifndef TYPE_VARIABLES_HPP
#define TYPE_VARIABLES_HPP
#include <eigen3/Eigen/Dense>
#include "tree.hh"
#include "tree_util.hh"
#include "Clothoid.h"
#include "dubins.h"

using namespace std;

typedef struct type_road_point
{
    double x;
    double y;
    double latitude;
    double longitude;
    double angle;
    double speed = 0;
    bool size = false;
    /* 0 means free, 1 means transit (can pass but no stop), 
       2 means stop (no passable), 3 means unsafe */
    int state = 0; 
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
    bool size = false;
    /* 0 means free, 1 means transit (can pass but no stop), 
       2 means stop (no passable), 3 means unsafe */
    int state = 0; 
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
    bool size = false; //for checking if it has initialized
}vehicle_velocity;

typedef struct GridMap
{
    vector<int> data;
    unsigned long width;
    unsigned long height;
    double resolution;
    bool size = false;
}GridMap;

typedef struct Speed
{
    double speed = 15;
    bool size = false; //for checking if it has been manual initialized
}Speed;

#endif // TYPE_VARIABLES_HPP
