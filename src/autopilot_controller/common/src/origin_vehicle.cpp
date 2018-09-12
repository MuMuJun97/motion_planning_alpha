#include "origin_vehicle.h"

#include <cmath>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// 车辆参数
double origin_vehicle::STEERING_R = 0.0;
double origin_vehicle::WHEEL_BASE = 0.0;
double origin_vehicle::CAR_LENGTH = 0.0;
double origin_vehicle::CAR_WIDTH = 0.0;
double origin_vehicle::CAR_HIGHT = 0.0;
double origin_vehicle::CAR_WEIGHT = 0.0;
double origin_vehicle::CAR_MIN_H = 0.0;

int origin_vehicle::MAX_STEERING_ANGLE = 0;
int origin_vehicle::MIN_STEERING_ANGLE = 0;
double origin_vehicle::RIGHT_STEERING_RATIO = 0.0;
double origin_vehicle::LEFT_STEERING_RATIO = 0.0;
double origin_vehicle::STEERING_RLR = 0.0;

double origin_vehicle::CORNERING_FRONT = 0.0;
double origin_vehicle::CORNERING_REAR = 0.0;

double origin_vehicle::LENGTH_A = 0.0;
double origin_vehicle::LENGTH_B = 0.0;

double origin_vehicle::SAFE_WIDTH = 0.0;
double origin_vehicle::SAFE_LENGTH = 0.0;

int origin_vehicle::MAP_BUFFER_PADDING_S = 0.0;

void origin_vehicle::loadCarPara(const std::string &filename) {
    boost::property_tree::ptree m_pt;
    boost::property_tree::ini_parser::read_ini(filename, m_pt);

    STEERING_R = m_pt.get<double>("STEERING_R", 0.0);
    WHEEL_BASE = m_pt.get<double>("WHEEL_BASE", 0.0);
    CAR_LENGTH = m_pt.get<double>("CAR_LENGTH", 0.0);
    CAR_WIDTH = m_pt.get<double>("CAR_WIDTH", 0.0);
    CAR_HIGHT = m_pt.get<double>("CAR_HIGHT", 0.0);
    CAR_WEIGHT = m_pt.get<double>("CAR_WEIGHT", 0.0);
    CAR_MIN_H = m_pt.get<double>("CAR_MIN_H", 0.0);

    MAX_STEERING_ANGLE = m_pt.get<int>("MAX_STEERING_ANGLE", 0);
    MIN_STEERING_ANGLE = m_pt.get<int>("MIN_STEERING_ANGLE", 0);
    RIGHT_STEERING_RATIO = m_pt.get<double>("RIGHT_STEERING_RATIO", 0.0);
    LEFT_STEERING_RATIO = m_pt.get<double>("LEFT_STEERING_RATIO", 0.0);
    // STEERING_RLR = m_pt.get<double>("STEERING_RLR", 0.0);
    STEERING_RLR = fabs(MAX_STEERING_ANGLE / MIN_STEERING_ANGLE);

    CORNERING_FRONT = m_pt.get<double>("CORNERING_FRONT", 0.0);
    CORNERING_REAR = m_pt.get<double>("CORNERING_REAR", 0.0);

    LENGTH_A = m_pt.get<double>("LENGTH_A", 0.0);
    LENGTH_B = m_pt.get<double>("LENGTH_B", 0.0);

    SAFE_WIDTH = m_pt.get<double>("SAFE_WIDTH", 0.0);
    SAFE_LENGTH = m_pt.get<double>("SAFE_LENGTH", 0.0);

    MAP_BUFFER_PADDING_S = m_pt.get<double>("MAP_BUFFER_PADDING_S", 0.0);
}
