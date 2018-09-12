#ifndef _PATH_H__
#define _PATH_H__

#include <vector>
#include "NaviPoint.hpp"
#include "Geography.hpp"
#include "GaussLocalGeographicCS.hpp"

class Path {
protected:
    static Geography::CSTransfer<GaussLocalGeographicCS> csTransfer;
public:
    void read_navi_file(const std::string &path_gps_log_file);

public:
    std::vector<NaviPoint> ref_points;

    int size() const;

    void cau_all_mileage_of_lane();

};

#endif /*_PATH_H__*/
