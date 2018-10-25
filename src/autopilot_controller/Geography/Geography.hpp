#ifndef _GEOGRAPHY_HPP__
#define _GEOGRAPHY_HPP__

#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cassert>

constexpr double to_radians(const double deg) { return (deg) * (M_PI / 180.0); }

constexpr double to_degrees(const double rad) { return (rad) * (180.0 / M_PI); }

inline double normalizeRad(double rad) {
    rad = fmod(rad, 2.0 * M_PI);
    return rad < 0.0 ? rad + 2.0 * M_PI : rad;
}

inline double normalizeDegree(double degree) {
    degree = fmod(degree, 360.0);
    return degree < 0.0 ? degree + 360.0 : degree;
}

namespace Geography {

    inline double right_direction(const double direction) {
        return normalizeDegree(direction + 90.0);
    }

    inline double left_direction(const double direction) {
        return normalizeDegree(direction - 90.0);
    }

    inline double inverse_direction(const double direction) {
        return normalizeDegree(direction + 180.0);
    }

    inline double xy_heading(double x0, double y0, double x1, double y1) {
        double xxx;
        double xx = (x1 - x0);
        double yy = (y1 - y0);

        double length = sqrt(xx * xx + yy * yy);

        if (length == 0)
            length = 0.000000001;

        double theta = 0.0;
        double theta_d = 0.0;

        xxx = fabs(xx);

        if (xx > 0.0 && yy > 0.0) { // 1
            theta = asin(xxx / length);
            theta_d = to_degrees(theta);
        }
        if (xx < 0.0 && yy > 0.0) { //4
            theta = asin(xxx / length);
            theta_d = 360.0 - to_degrees(theta);
        }
        if (xx > 0.0 && yy < 0.0) { //2
            theta = asin(xxx / length);
            theta_d = 180.0 - to_degrees(theta);
        }
        if (xx < 0.0 && yy < 0.0) { //3
            theta = asin(xxx / length);
            theta_d = 180.0 + to_degrees(theta);
        }

        if (xx == 0.0 && yy > 0.0)
            theta_d = 0.0;
        if (xx == 0.0 && yy < 0.0)
            theta_d = 180.0;
        if (xx > 0.0 && yy == 0.0)
            theta_d = 90.0;
        if (xx < 0.0 && yy == 0.0)
            theta_d = 270.0;

        return theta_d;
    }

    inline double xy_distance(const double x0, const double y0, const double x1, const double y1) {
        return hypot(x1 - x0, y1 - y0);
    }

    inline double xy_mdistance(const double x0, const double y0, const double x1, const double y1) {
        return fabs(x1 - x0) + fabs(y1 - y0);
    }

    template<class LocalGeographicCS>
    class CSTransfer {
        LocalGeographicCS *cs;
        double referenceLat0;
        double referenceLon0;

    public:
        double getReferenceLat0() const { return referenceLat0; }

        double getReferenceLon0() const { return referenceLon0; }

        double ll_heading(double lat0, double lon0, double lat1, double lon1) {
            assert(cs->isDescartesCoordinates);

            double x0, y0, x1, y1;
            ll2xy(lat0, lon0, x0, y0);
            ll2xy(lat1, lon1, x1, y1);
            return xy_heading(x0, y0, x1, y1);
        }

        double ll_distance(const double lat0, const double lon0, const double lat1, const double lon1) {
            assert(cs->isDescartesCoordinates);

            double x0, y0, x1, y1;
            ll2xy(lat0, lon0, x0, y0);
            ll2xy(lat1, lon1, x1, y1);
            return hypot(x1 - x0, y1 - y0);
        }

        double ll_mdistance(const double lat0, const double lon0, const double lat1,
                            const double lon1) {
            assert(cs->isDescartesCoordinates);

            double x0, y0, x1, y1;
            ll2xy(lat0, lon0, x0, y0);
            ll2xy(lat1, lon1, x1, y1);
            return fabs(x1 - x0) + fabs(y1 - y0);
        }

        void llh2xyz(const double lat, const double lon, const double height,
                     double &x, double &y, double &z) {
            cs->llh2xyz(lat, lon, height, x, y, z);
        }

        void xyz2llh(const double x, const double y, const double z,
                     double &lat, double &lon, double &height) {
            cs->xyz2llh(x, y, z, lat, lon, height);
        }

        void ll2xy(const double lat, const double lon, double &x, double &y) {
            assert(cs->isDescartesCoordinates);

            double z;
            cs->llh2xyz(lat, lon, 0.0, x, y, z);
        }

        void xy2ll(const double x, const double y, double &lat, double &lon) {
            assert(cs->isDescartesCoordinates);

            double height;
            cs->xyz2llh(x, y, 0.0, lat, lon, height);
        }

        // for no origin point xyz
        void subXYZOrigin(double &x, double &y, double &z) {
            cs->subXYZOrigin(x, y, z);
        }

        CSTransfer() {
            char line[256];
            std::string workpathvar = ros::package::getPath("autopilot_controller");
            if (workpathvar.empty()) {
                ROS_ERROR("Finding Working Path Failed");
                exit(-1);
            }
            FILE *localfile = fopen((workpathvar + "/config/local.txt").c_str(), "r");
            if (!localfile) {
                ROS_ERROR("Could not open %s/config/local.txt", workpathvar.c_str());
                exit(-1);
            }

            while (fgets(line, sizeof(line), localfile) != NULL) {
                if (line[0] != '#') {
                    sscanf(line, "%lf %lf", &referenceLat0, &referenceLon0);
                    if (true) { //referenceLon0 > 60.0 && referenceLon0 < 150.0
                        cs = new LocalGeographicCS(referenceLat0, referenceLon0);
                        return;
                    } else
                        break;
                }
            }

            fclose(localfile);
            fprintf(stderr, "Geography::CSTransfer: cannot load lat0, lon0\n");
            exit(-1);
        }

        virtual ~CSTransfer() {
            if (cs) {
                delete cs;
                cs = nullptr;
            }
        }
    };

}

#endif /*_GEOGRAPHY_HPP__*/
