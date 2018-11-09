#ifndef SAMPLING_METHODS_H
#define SAMPLING_METHODS_H

//#include <type_global_variable.hpp>
//#include <function_methods.h>
#include "function_simplified.hpp"
using namespace func_simplified;
class sampling_methods
{
public:
    sampling_methods();
public:
    /*
     * sample the sample node in the working space uniformly.
     */
    bool uniform_sampling_method();
    /*
     * sample the sample node near by the reference path.
     * methods can be refered from the paper
     */
    bool sampling_nearby_reference_path(std::vector<type_road_point> reference_path);
    /*
     * sampling the sample node in position of interest.
     */
    bool sampling_interesting_area();
    /*
     * tune sampling parameters
     *
     * /
     */
    void tune_sampling_parameters(double dist,double D_dist);
public:
    /*
     * got a sample node from the work space.
     */
    type_road_point sample_node;
    /*
     * three parameter used in the sampling process.
     * R0 represent the area around the points in the reference path, but not sampling.
     * Sigma_R represent the distribution extend of radius.
     * Sigma_T represent the distribution extend of angle;
     */
    double R0,Sigma_R,Sigma_T;
    fun_simple** p_func_from_sampling;

    double nRADIUS_0 = 2.7;
    double nSIGMA_R = 1.4;

    double nTHETA_0 = 0;
    double nSIGMA_T = 0.4;

    double nHEADING_0 = 0;
    double nSIGMA_H = M_PI / 48;

    double uRADIUS_0 = 3.3;
    double uSIGMA_R = 1.1;

    double uTHETA_0 = - M_PI / 2;
    double uSIGMA_T = M_PI / 8;

    double UNSAFE_ATTENTION = 0.6;
    double NORMAL_ATTENTION = 0.4;

    double uHEADING_0 = 0;
    double uSIGMA_H = M_PI / 24;

    int THRESHOLD = 10;

private:
    /*
     * engine for the random sample.
     */
    std::random_device ran_device;

};

#endif // SAMPLING_METHODS_H
