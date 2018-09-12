#include <fstream>
#include <iostream>
#include <vector>
#include <cstdlib>

#include "math_util.h"
#include "NaviPoint.hpp"
#include "origin_vehicle.h"
#include "Path.hpp"

Geography::CSTransfer<GaussLocalGeographicCS> Path::csTransfer;

void Path::read_navi_file(const std::string &path_gps_log_file) {
    std::ifstream inFiles(path_gps_log_file, std::ios::in);
    if (!inFiles)
        std::cerr << "can not read ins file" << std::endl;

    double total_s = 0.0;
    double heading;
    double baseLat, baseLon;
    double lat, lon;
    std::string wayname;
    std::string tmp[4];
    unsigned int size;

    ref_points.clear();

    NaviPoint nd;
    while (inFiles) {
        inFiles >> wayname;
        inFiles >> tmp[0] >> tmp[1] >> tmp[2] >> tmp[3];

        inFiles >> baseLat >> baseLon;
        if ((fabs(baseLat) > 0.1 && fabs(baseLon) > 0.1)
            && csTransfer.ll_distance(baseLat, baseLon, csTransfer.getReferenceLat0(),
                                      csTransfer.getReferenceLon0()) > 0.2) {
            std::cerr << "Path::read_navi_file: GPS path \"" << wayname
                      << "\" is out of date, different from current DGPS base station position. "
                      << "Current base station position is (" << csTransfer.getReferenceLat0() << ", "
                      << csTransfer.getReferenceLon0() << "), " << "while path uses (" << baseLat
                      << ", " << baseLon << ") as base station position" << std::endl;
            exit(-1);
        }

        inFiles >> size;

        for (unsigned int i = 0; i < size; i++) {
            inFiles >> lat >> lon >> heading >> total_s;

            csTransfer.ll2xy(lat, lon, nd.position_x, nd.position_y);
            nd.position_z = 0.0;

            nd.speed_desired_Uxs = 15.0;
            nd.heading = heading;
            nd.steering_ks = 0.0;

            nd.s = total_s;

            ref_points.push_back(nd);
        }
    }

    inFiles.close();

    ref_points.pop_back();
}

int Path::size() const {
    return static_cast<int>(ref_points.size());
}

void Path::cau_all_mileage_of_lane() {
    NaviPoint p0;
    p0 = ref_points[0];

    double x = p0.position_x;
    double y = p0.position_y;
    double xx, yy, ss;

    double s = 0;
    ref_points[0].s = 0;

    for (unsigned int i = 1; i < ref_points.size(); i++) {
        xx = ref_points[i].position_x;
        yy = ref_points[i].position_y;

        ss = sqrt((xx - x) * (xx - x) + (yy - y) * (yy - y));
        s += ss;

        ref_points[i].s = s;

        assert(s >= 0);
        assert(s >= ref_points[i - 1].s);
        assert(s <= ref_points[i - 1].s + 60);

        x = xx;
        y = yy;
    }

}
