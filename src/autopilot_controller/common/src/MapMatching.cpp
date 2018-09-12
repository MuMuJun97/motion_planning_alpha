#include <vector>
#include <iostream>
#include "MapMatching.hpp"

using namespace std;

MapMatching::MapMatching() {
    init();
}

void MapMatching::init() {
    last_match_point_no = -1;
    current_match_point_no = 0;
    next_match_point_no = 0;
}

int MapMatching::find_next_moition_planning_points(const Path &p, const double length) {
    double s = 0;
    double x, y, x_c, y_c;

    if (p.ref_points.size() <= 0)
        return 0;

    int match_point = current_match_point_no;

    x = (p.ref_points[match_point]).position_x;
    y = (p.ref_points[match_point]).position_y;

    // #pragma omp parallel for
    while (s < length) {
        x_c = (p.ref_points[match_point + 1]).position_x;
        y_c = (p.ref_points[match_point + 1]).position_y;

        s += sqrt((x_c - x) * (x_c - x) + (y_c - y) * (y_c - y));

        x = x_c, y = y_c;

        match_point += 10;  // for boost the search speed
        //    match_point ++ ;  // for boost the search speed

        if ((match_point - current_match_point_no) > 5000)
            break;
    }

    return match_point;
}

int MapMatching::MapMarch_Min_Distance_motion_planning(double Current_X, double Current_Y,
                                                       double Current_Heading, const Path &p, double &min_error) {
    double x, y, Temp_X, Temp_Y;
    double diff_heading;

    int count = 0;
    int flag = 0;

    double length_min = 8888;
    double length_C = 9999;

    if (p.ref_points.size() <= 0)
        return 0;

    if (last_match_point_no == -1)
        count = 0;
    else
        count = last_match_point_no;

//    if(last_match_point_no > 100)
//       last_match_point_no -=100;

    x = Current_X;
    y = Current_Y;

    // 粗搜索定位大概位置, 搜索步长为10
    //#pragma omp parallel for
    for (; (count < last_match_point_no + 50000) && (count + 10 < p.size()); count += 10) {

        Temp_X = (p.ref_points[count]).position_x;
        Temp_Y = (p.ref_points[count]).position_y;

        length_C = (Temp_X - x) * (Temp_X - x) + (Temp_Y - y) * (Temp_Y - y);
        length_C = sqrt(length_C);

        if (length_min > length_C) {
            length_min = length_C;
            flag = count;
        }

        //考虑到定位误差，如果在3m之内已经很好了，可以推出搜索。
        if (length_min < 3) {

            diff_heading = fabs(Current_Heading - p.ref_points[count].heading);

            if (diff_heading > 180.0)
                diff_heading = 360.0 - diff_heading;

            if (diff_heading > 90.0)
                continue;

            length_min = length_C;
            flag = count;

            break;
        }

    }

//    cout << " first lenght_min " << length_min
//         << " sqrt " << sqrt(length_min)
//         << " current_match_point_no " << flag
//         << " count " << count << endl;

    last_match_point_no = flag;
    current_match_point_no = flag;

    //  if(count > 100)
    //      count -= 100;

    //精细搜索，定位到具体位置。
    //  #pragma omp parallel for
    for (; (count < last_match_point_no + 600) && (count < p.size()); count++) {
        Temp_X = (p.ref_points[count]).position_x;
        Temp_Y = (p.ref_points[count]).position_y;

        length_C = (Temp_X - x) * (Temp_X - x) + (Temp_Y - y) * (Temp_Y - y);
        length_C = sqrt(length_C);

        if (count == 0) {
            length_min = length_C;
        } else if (length_min > length_C) {
            length_min = length_C;
            flag = count;
        }

        //考虑到定位误差，如果在10cm之内已经很好了，可以推出搜索。
        if (length_min < 0.01)
            break;
    }

    //    cout << " lenght_min " << length_min
    //           << " current_match_point_no " << flag
    //           << " last_match_point_no " << flag << endl;

    min_error = length_min;

    last_match_point_no = flag;
    current_match_point_no = flag;

    return current_match_point_no;
}
