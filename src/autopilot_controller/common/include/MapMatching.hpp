#ifndef _MAPMATCHING_HPP__
#define _MAPMATCHING_HPP__

#include <cmath>

#include "Path.hpp"

struct MapMatching {
    // 最近的匹配点中的相关信息。
    int last_match_point_no;
    int next_match_point_no;

    int current_match_point_no;

    MapMatching();

    void init();

    // 地图匹配算法
    int MapMarch_Min_Distance_motion_planning(double Current_X, double Current_Y,
                                              double Current_Heading, const Path &p, double &min_error);

    int find_next_moition_planning_points(const Path &p, const double length);
};

#endif /*_MAPMATCHING_HPP__*/
