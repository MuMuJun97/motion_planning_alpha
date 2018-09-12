#ifndef _ORIGIN_VEHICLE_H__
#define _ORIGIN_VEHICLE_H__

#include <string>

struct origin_vehicle {
    const static int NO_LANE = -1;
    const static int MIDDLE_LANE = 0;
    const static int LEFT_LANE = 1;
    const static int RIGHT_LANE = 2;

    // 车辆参数部分
    static double STEERING_R;
    static double WHEEL_BASE; //轴距
    static double CAR_LENGTH;
    static double CAR_WIDTH;
    static double CAR_HIGHT;
    static double CAR_WEIGHT;  //kg
    static double CAR_MIN_H;

    // MAX STEERING ANGLE WHEN TURN RIGHT
    static int MAX_STEERING_ANGLE;
    // MAX STEERING ANGLE WHEN TURN LEFT
    static int MIN_STEERING_ANGLE;
    static double RIGHT_STEERING_RATIO;
    static double LEFT_STEERING_RATIO;
    // STEERING_RLR = abs(MAX_STEERING_ANGLE / MIN_STEERING_ANGLE)
    static double STEERING_RLR;

    static double CORNERING_FRONT;  // Front cornering stiffness
    static double CORNERING_REAR;   // Rear cornering stiffness

    static double LENGTH_A;  // Front axle to CG distance
    static double LENGTH_B;  // Rear axle to CG distance

    static double SAFE_WIDTH;   //安全保护的距离
    static double SAFE_LENGTH;  //安全保护的距离

    static int MAP_BUFFER_PADDING_S;

    static void loadCarPara(const std::string &filename);
};

#endif /*_ORIGIN_VEHICLE_H__*/
