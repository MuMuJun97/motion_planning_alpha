#include <fstream>
#include <iostream>
#include <cmath>
#include <cassert>

#include "math_util.h"
#include "spline.h"
#include "heading.h"
#include "steering_angle.h"
#include "cau_heading_steering.h"
#include "Path.hpp"

#define BASE_SPEED_ (4.0f)

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//   double spline cau steering angle and heading
//
/////////////////////////////////////////////////////////////////////////////////////////////////

/*
 void cau_output_from_double_spline(path& p, int no_points, path ref) {
 if (no_points < 150)
 return;

 int i, j;
 // int head_free, tail_free;

 std::vector<double> x, y, s;

 tk::spline s_x1, s_y1;
 tk::spline s_x2, s_y2;

 double steering_ks, steering_r;
 double heading_ks, heading_r;
 double Ux_ks;

 double cs, r;

 //    ofstream outfile("ste_output.txt", std::ios::app);
 //    outfile.precision(6);

 for (i = 0; i < no_points - 1500; i += 1000) {
 // 750
 x.clear();
 y.clear();
 s.clear();

 for (j = 0; (j <= 1000) && (j + i < no_points); j += 100) {
 x.push_back(p.ref_points[j + i].position_x);
 y.push_back(p.ref_points[j + i].position_y);
 s.push_back(p.ref_points[j + i].s);
 }

 s_x1.set_points(s, x);
 s_y1.set_points(s, y);

 for (j = 200; (j < 600) && (j + i < no_points); j++) {
 cs = p.ref_points[j + i].s;
 Ux_ks = ref.ref_points[j + i].longitudinal_speed;
 heading_r = ref.ref_points[j + i].heading;
 steering_r = ref.ref_points[j + i].steering_angle;

 heading_ks = cau_heading_angle_from_ks(s_x1, s_y1, cs, 3);
 steering_ks = cau_steering_angle_from_ks(s_x1, s_y1, cs, Ux_ks, r,
 0.3, 0.6);

 p.ref_points[j + i].heading = heading_ks;
 p.ref_points[j + i].steering_angle = steering_ks;

 //     outfile << " cs " <<  cs
 //             << " steering_r" <<  steering_r
 //             << " st_ks " << steering_ks
 //             << " error " << (steering_ks - steering_r)
 //             << endl;
 }

 x.clear();
 y.clear();
 s.clear();

 for (j = 500; (j <= 1500) && (j + i < no_points); j += 100) {
 x.push_back(p.ref_points[j + i].position_x);
 y.push_back(p.ref_points[j + i].position_y);
 s.push_back(p.ref_points[j + i].s);
 }

 s_x2.set_points(s, x);
 s_y2.set_points(s, y);

 for (j = 700; (j < 1200) && (j + i < no_points); j++) {
 cs = p.ref_points[j + i].s;
 Ux_ks = ref.ref_points[j + i].longitudinal_speed;
 heading_r = ref.ref_points[j + i].heading;
 steering_r = ref.ref_points[j + i].steering_angle;

 heading_ks = cau_heading_angle_from_ks(s_x2, s_y2, cs, 3);
 steering_ks = cau_steering_angle_from_ks(s_x2, s_y2, cs, Ux_ks, r,
 0.3, 0.6);

 p.ref_points[j + i].position_x = s_x1(cs);
 p.ref_points[j + i].position_y = s_y1(cs);

 p.ref_points[j + i].heading = heading_ks;
 p.ref_points[j + i].steering_angle = steering_ks;

 //       outfile << " cs " <<  cs
 //               << " steering_r" <<  steering_r
 //               << " st_ks " << steering_ks
 //               << " error " << (steering_ks - steering_r)
 //               << endl;
 }

 x.clear();
 y.clear();
 s.clear();

 for (j = 500; (j <= 1500) && (j + i < no_points); j += 100) {
 x.push_back(p.ref_points[j + i].position_x);
 y.push_back(p.ref_points[j + i].position_y);
 s.push_back(p.ref_points[j + i].s);
 }

 s_x2.set_points(s, x);
 s_y2.set_points(s, y);

 for (j = 700; (j < 1200) && (j + i < no_points); j++) {
 cs = p.ref_points[j + i].s;
 Ux_ks = ref.ref_points[j + i].longitudinal_speed;
 heading_r = ref.ref_points[j + i].heading;
 steering_r = ref.ref_points[j + i].steering_angle;

 heading_ks = cau_heading_angle_from_ks(s_x2, s_y2, cs, 3);
 steering_ks = cau_steering_angle_from_ks(s_x2, s_y2, cs, Ux_ks, r,
 0.3, 0.6);

 p.ref_points[j + i].position_x = s_x2(cs); //还有些问题。 主要是接头平顺性问题。
 p.ref_points[j + i].position_y = s_y2(cs);

 p.ref_points[j + i].heading = heading_ks;
 p.ref_points[j + i].steering_angle = steering_ks;

 //          outfile << " cs " <<  cs
 //                  << " steering_r" <<  steering_r
 //                  << " st_ks " << steering_ks
 //                  << " error " << (steering_ks - steering_r)
 //                  << endl;
 }

 }

 //   outfile.close();
 }

 //////////////////////////////////////////////////////////////////////////////////////////////////
 //
 //   single spline cau steering angle and heading
 //
 /////////////////////////////////////////////////////////////////////////////////////////////////

 void cau_output_from_single_spline(path& p, int no_points, path ref) {
 if (no_points < 150)
 return;

 int i, j, l, k;

 std::vector<double> x, y, s, h, st;
 std::vector<double> xx, yy, ss, hh, stst;

 tk::spline s_x1, s_y1;
 tk::spline s_x2, s_y2;

 struct spline_list control_splines;

 double steering_ks, steering_r;
 double heading_ks, heading_r;
 double Ux_ks;
 double cs, r;

 // 采集均匀的控制点
 k = 0;
 for (i = 0; i < no_points; i++) {
 if (p.ref_points[i].s > k * 4) {
 s.push_back(p.ref_points[i].s);
 x.push_back(p.ref_points[i].position_x);
 y.push_back(p.ref_points[i].position_y);
 k++;
 }
 }

 //里程的插值方法
 l = 0;
 for (k = 0; k < s.size() - 30; k += 20) {
 xx.clear();
 yy.clear();
 ss.clear();

 //0 40;
 for (j = 0; (j <= 20) && (j + k < s.size()); j++) {
 xx.push_back(x[j + k]);
 yy.push_back(y[j + k]);
 ss.push_back(s[j + k]);
 }

 s_x1.set_points(ss, xx);
 s_y1.set_points(ss, yy);

 control_splines.spline_x_list.push_back(s_x1);
 control_splines.spline_y_list.push_back(s_y1);

 //  control_splines.min_s.push_back(ss[k]+2);
 //  control_splines.max_s.push_back(ss[k+20]-2);
 //  control_splines.cent_s.push_back( (control_splines.min_s[l] + control_splines.max_s[l])/2 );

 l++;

 xx.clear();
 yy.clear();
 ss.clear();

 // 20 60
 for (j = 10; (j <= 30) && (j + k < s.size()); j++) {
 xx.push_back(x[j + k]);
 yy.push_back(y[j + k]);
 ss.push_back(s[j + k]);
 }

 s_x2.set_points(ss, xx);
 s_y2.set_points(ss, yy);

 control_splines.spline_x_list.push_back(s_x2);
 control_splines.spline_y_list.push_back(s_y2);

 //   control_splines.min_s.push_back(ss[k+10]+2);
 //  control_splines.max_s.push_back(ss[k+30]-2);
 //  control_splines.cent_s.push_back( (control_splines.min_s[l] + control_splines.max_s[l])/2);
 l++;
 }

 //   作一个索引来帮助查找。
 ofstream outfile("all_spline_st.txt", std::ios::app);
 outfile.precision(4);

 for (i = 0; i < no_points - 2000; i++) {
 cs = ref.ref_points[i].s;
 Ux_ks = ref.ref_points[i].longitudinal_speed;
 heading_r = ref.ref_points[i].heading;
 steering_r = ref.ref_points[i].steering_angle;

 l = (int) ((cs - 10) / 40);

 if (l < 0)
 l = 0;

 heading_ks = cau_heading_angle_from_ks(control_splines.spline_x_list[l],
 control_splines.spline_y_list[l], cs, 3);

 steering_ks = cau_steering_angle_from_ks(
 control_splines.spline_x_list[l],
 control_splines.spline_y_list[l], cs, Ux_ks, r, 0.3, 0.6);

 p.ref_points[j + i].heading = heading_ks;
 p.ref_points[j + i].steering_angle = steering_ks;

 outfile << " cs " << cs << "\t" << " st_r " << heading_r << "\t"
 << " st_ks " << heading_ks << "\t" << " e_h "
 << (heading_ks - heading_r) << "\t" << " st_r " << steering_r
 << "\t" << " st_ks " << steering_ks << "\t" << " e_st "
 << (steering_ks - steering_r) << "\t" << endl;

 }

 outfile.close();
 }



 //////////////////////////////////////////////////////////////////////////////////////////////////
 //
 //   realtime single spline cau steering angle and heading
 //
 /////////////////////////////////////////////////////////////////////////////////////////////////

 void cau_heading_steering_from_spline_realtime(path& p, int st_p, int end_p,
 double speed) {
 if (end_p - st_p < 50)
 return;

 int i, k;
 double Ux_ks;
 double cs, r;
 double steering_ks, steering_r;
 double heading_ks, heading_r;

 std::vector<double> x, y, s;
 tk::spline s_x1, s_y1;

 // 采集均匀的控制点
 k = 0;

 x.clear();
 y.clear();
 s.clear();

 int length = p.ref_points.size();
 double s0 = p.ref_points[0].s;
 double sl = 0;

 for (i = 0; i < length; i++) {
 sl = p.ref_points[i].s - s0;

 if (sl >= k * 1.0) {
 s.push_back(p.ref_points[i].s);
 x.push_back(p.ref_points[i].position_x);
 y.push_back(p.ref_points[i].position_y);
 k++;
 }
 }

 if (s.size() <= 2) {
 cout << " bad error length " << length << endl;
 for (i = 0; i < length; i++) {
 cerr << "  " << p.ref_points[i].s;

 }
 cerr << " end_p " << end_p << " st_p " << st_p << endl;

 }
 s_x1.set_points(s, x);
 s_y1.set_points(s, y);

 for (i = 0; i < length; i++) {
 cs = p.ref_points[i].s;
 Ux_ks = p.ref_points[i].longitudinal_speed;

 if (speed >= 0)
 Ux_ks = speed;

 heading_ks = cau_heading_angle_from_ks(s_x1, s_y1, cs, 3);

 steering_ks = cau_steering_angle_from_ks(s_x1, s_y1, cs, Ux_ks, r, 0.3,
 0.6);

 p.ref_points[i].heading = heading_ks;
 p.ref_points[i].steering_angle = steering_ks;

 }

 }
 */

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//  only single spline cau steering angle and heading
//
/////////////////////////////////////////////////////////////////////////////////////////////////
/*void cau_all_output_from_single_spline(path& p, int no_points, path ref) {
 if (no_points < 150)
 return;

 const double Ksteering = 0.01;
 const double Kspeed = 0.15;

 std::vector<double> x, y, s, h, st;
 std::vector<double> xx, yy, ss, hh, stst;

 tk::spline s_x1, s_y1;

 struct spline_list control_splines;

 double steering_ks;
 double heading_ks;
 double Ux_ks;
 double cs, r;

 // 采集均匀的控制点
 int i;
 int k = 0;
 for (i = 0; i < no_points; i++) {
 if (p.ref_points[i].s > k * 8) {
 s.push_back(p.ref_points[i].s);
 x.push_back(p.ref_points[i].position_x);
 y.push_back(p.ref_points[i].position_y);
 k++;
 }
 }

 //里程的插值方法
 s_x1.set_points(s, x);
 s_y1.set_points(s, y);

 for (i = 0; i < no_points - 500; i++) {
 cs = ref.ref_points[i].s;
 Ux_ks = ref.ref_points[i].longitudinal_speed;

 heading_ks = cau_heading_angle_from_ks(s_x1, s_y1, cs, 3);

 steering_ks = cau_steering_angle_from_ks(s_x1, s_y1, cs, Ux_ks, r, 0.3,
 0.6);

 steering_ks = iclamp(steering_ks, origin_vehicle::MIN_STEERING_ANGLE,
 origin_vehicle::MAX_STEERING_ANGLE);

 p.ref_points[i].heading = heading_ks;
 p.ref_points[i].steering_angle = steering_ks;
 p.ref_points[i].position_x = s_x1(cs);
 p.ref_points[i].position_y = s_y1(cs);

 double abs_steering_ks = fabs(steering_ks);

 if (i < 500) {
 p.ref_points[i].speed_desired_Uxs = BASE_SPEED_;
 } else if (i < no_points - 500) {

 if (abs_steering_ks
 > 18.0&& p.ref_points[i].speed_desired_Uxs > BASE_SPEED_) {

 navi_point& slow_point = p.ref_points[i - 500];
 double ratio = pow(
 (2
 + Kspeed
 * (p.ref_points[i].speed_desired_Uxs
 - BASE_SPEED_)),
 -Ksteering * abs_steering_ks);

 slow_point.speed_desired_Uxs = BASE_SPEED_
 + (p.ref_points[i].speed_desired_Uxs - BASE_SPEED_)
 * ratio;

 cs = slow_point.s;
 steering_ks = cau_steering_angle_from_ks(s_x1, s_y1, cs,
 slow_point.speed_desired_Uxs, r, 0.3, 0.6);
 steering_ks = iclamp(steering_ks,
 origin_vehicle::MIN_STEERING_ANGLE,
 origin_vehicle::MAX_STEERING_ANGLE);
 slow_point.steering_angle = steering_ks;
 }
 } else {
 p.ref_points[i].speed_desired_Uxs = 0.0;
 }
 }

 }*/
int cau_all_output_from_single_spline(Path &p, int no_points, double desired_Ux) {
    if (no_points < 100)
        return 0;

    const double Ksteering = 0.006; // 0.01
    const double Kspeed = 0.15;     // 0.13

    std::vector<double> x, y, s;
    std::vector<double> speed;

    tk::spline s_x1, s_y1;

    double steering_ks;
    double heading_ks;
    double Ux_ks;
    double cs, r;

    double abs_steering_ks;

    // 采集均匀的控制点
    double last_s = -1000.0;
    double s_step = 8.0;
    for (int i = 1; i < no_points - 1; i++) {
        if (p.ref_points[i].s - last_s >= s_step) {
            const NaviPoint &prev_p = p.ref_points[i - 1];
            const NaviPoint &curr_p = p.ref_points[i];
            const NaviPoint &next_p = p.ref_points[i + 1];
            s.push_back((prev_p.s + curr_p.s + next_p.s) / 3.0);
            x.push_back((prev_p.position_x + curr_p.position_x + next_p.position_x) / 3.0);
            y.push_back((prev_p.position_y + curr_p.position_y + next_p.position_y) / 3.0);
            last_s = p.ref_points[i].s;
        }
    }

    //里程的插值方法
    s_x1.set_points(s, x);
    s_y1.set_points(s, y);

    const double total_s = p.ref_points.back().s;

    assert(total_s < 50000.0 && total_s > 0.0);
    p.ref_points.resize((unsigned int) (total_s * 10.0 + 20.0));
    speed.resize((unsigned int) (total_s * 10.0 + 20.0));
    s.resize((unsigned int) (total_s * 10.0 + 20.0));

    unsigned int i = 0;
    for (cs = 0; cs <= total_s; cs += 0.1, i++) {
        Ux_ks = desired_Ux;

        heading_ks = cau_heading_angle_from_ks(s_x1, s_y1, cs, 3.0);

        steering_ks = cau_steering_angle_from_ks(s_x1, s_y1, cs, Ux_ks, r, 0.3, 0.6);

        steering_ks = iclamp(steering_ks, origin_vehicle::MIN_STEERING_ANGLE,
                             origin_vehicle::MAX_STEERING_ANGLE);

        NaviPoint &spline_point = p.ref_points[i];

        spline_point.position_x = s_x1(cs);
        spline_point.position_y = s_y1(cs);
        spline_point.position_z = 0.0;

        spline_point.speed_desired_Uxs = Ux_ks;

        spline_point.heading = heading_ks;
        spline_point.steering_ks = steering_ks;

        spline_point.s = cs;
//        spline_point.lateral_offset = 0.0;
//        spline_point.k_s = 1.0 / r;

        if (cs >= 15.0 && cs < total_s - 8.0) {
            abs_steering_ks = fabs(steering_ks);

            if (abs_steering_ks > 13.0 && desired_Ux > BASE_SPEED_) {
                NaviPoint &slow_point_before = p.ref_points[i - 150];
                NaviPoint &slow_point_after = p.ref_points[i + 80];

                double ratio = pow((2.0 + Kspeed * (spline_point.speed_desired_Uxs - BASE_SPEED_)),
                                   -Ksteering * abs_steering_ks);

                if (std::isnan(ratio) || ratio < 0.0)
                    ratio = 0.0;
                else if (ratio > 1.0)
                    ratio = 1.0;

                spline_point.speed_desired_Uxs = BASE_SPEED_
                                                 + (spline_point.speed_desired_Uxs - BASE_SPEED_) * ratio;

                slow_point_before.speed_desired_Uxs = slow_point_after.speed_desired_Uxs =
                        spline_point.speed_desired_Uxs;
            }
        } else if (cs >= total_s - 8.0)
            spline_point.speed_desired_Uxs = 0.0;

        s[i] = cs;
        speed[i] = spline_point.speed_desired_Uxs;
    }

    p.ref_points.resize(i);
    s.resize(i);
    speed.resize(i);

    tk::spline s_speed1;
    s_speed1.set_points(s, speed);

    for (i = 0; i < p.ref_points.size(); ++i)
        p.ref_points[i].speed_desired_Uxs = s_speed1(p.ref_points[i].s);

    return p.size();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//  realtime single spline cau steering angle and heading
//
/////////////////////////////////////////////////////////////////////////////////////////////////

int cau_all_output_from_single_spline_realtime(Path &p, int no_points) {
    if (no_points < 10)
        return 0;

    const double Ksteering = 0.006;
    const double Kspeed = 0.15;

    std::vector<double> x, y, s;
    std::vector<double> speed;

    tk::spline s_x1, s_y1;
    tk::spline s_speed1;

    double steering_ks;
    double heading_ks;
    double Ux_ks;
    double cs, r;

    double abs_steering_ks;

    // 采集均匀的控制点
    double curr_s, last_s = -1000.0;
    double s_step = 4.0;
    while (s.size() <= 2) {
        for (int i = 1; i < no_points - 1; i++) {
            if (p.ref_points[i].s - last_s >= s_step) {
                const NaviPoint &prev_p = p.ref_points[i - 1];
                const NaviPoint &curr_p = p.ref_points[i];
                const NaviPoint &next_p = p.ref_points[i + 1];
                curr_s = (prev_p.s + curr_p.s + next_p.s) / 3.0;
                if (curr_s <= last_s)
                    continue;

                s.push_back(curr_s);
                x.push_back((prev_p.position_x + curr_p.position_x + next_p.position_x) / 3.0);
                y.push_back((prev_p.position_y + curr_p.position_y + next_p.position_y) / 3.0);
                speed.push_back(
                        (prev_p.speed_desired_Uxs + curr_p.speed_desired_Uxs
                         + next_p.speed_desired_Uxs) / 3.0);
                last_s = curr_s;
            }
        }
        s_step *= 0.5;
        if (s_step < 0.5)
            return 0;
    }

    if (s.size() <= 2)
        return 0;

    //里程的插值方法
    s_x1.set_points(s, x);
    s_y1.set_points(s, y);
    s_speed1.set_points(s, speed);

    const double total_s = p.ref_points.back().s;

    p.ref_points.resize((unsigned int) (total_s * 10.0 + 20.0));
    speed.resize((unsigned int) (total_s * 10.0 + 20.0));
    s.resize((unsigned int) (total_s * 10.0 + 20.0));

    unsigned int i = 0;
    for (cs = 0.0; cs <= total_s; cs += 0.1, i++) {
        Ux_ks = s_speed1(cs);

        heading_ks = cau_heading_angle_from_ks(s_x1, s_y1, cs, 3.0);
        steering_ks = cau_steering_angle_from_ks(s_x1, s_y1, cs, Ux_ks, r, 0.75, 1.5);

        steering_ks = iclamp(steering_ks, origin_vehicle::MIN_STEERING_ANGLE,
                             origin_vehicle::MAX_STEERING_ANGLE);

        NaviPoint &spline_point = p.ref_points[i];

        spline_point.position_x = s_x1(cs);
        spline_point.position_y = s_y1(cs);
        spline_point.position_z = 0.0;

        spline_point.heading = heading_ks;
        spline_point.steering_ks = steering_ks;
        spline_point.speed_desired_Uxs = Ux_ks;

        spline_point.s = cs;
//        spline_point.lateral_offset = 0.0;
//        spline_point.k_s = 1.0 / r;

        if (cs >= 3.0 && cs < total_s - 5.0) {
            abs_steering_ks = fabs(steering_ks);

            if (abs_steering_ks > 13.0 && Ux_ks > BASE_SPEED_) {
                NaviPoint &slow_point_before = p.ref_points[i - (i >= 50 ? 50 : 30)];
                NaviPoint &slow_point_after = p.ref_points[i + 50];

                double ratio = pow((2.0 + Kspeed * (spline_point.speed_desired_Uxs - BASE_SPEED_)),
                                   -Ksteering * abs_steering_ks);

                if (std::isnan(ratio) || ratio < 0.0)
                    ratio = 0.0;
                else if (ratio > 1.0)
                    ratio = 1.0;

                spline_point.speed_desired_Uxs = BASE_SPEED_
                                                 + (spline_point.speed_desired_Uxs - BASE_SPEED_) * ratio;

                slow_point_before.speed_desired_Uxs = slow_point_after.speed_desired_Uxs =
                        spline_point.speed_desired_Uxs;

            }
        } else if (cs >= total_s - 5.0)
            spline_point.speed_desired_Uxs = 0.0;

        s[i] = cs;
        speed[i] = spline_point.speed_desired_Uxs;
    }
    p.ref_points.resize(i);
    s.resize(i);
    speed.resize(i);

    if (s.size() <= 2)
        return 0;

    s_speed1.set_points(s, speed);

    for (i = 0; i < p.ref_points.size(); ++i)
        p.ref_points[i].speed_desired_Uxs = s_speed1(p.ref_points[i].s);

    return p.size();
}
