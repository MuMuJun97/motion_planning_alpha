#ifndef _CONTROLLERWATCHDOG_HPP__
#define _CONTROLLERWATCHDOG_HPP__

#include <map>
#include <string>
#include <pthread.h>

#include "steering_control_info.hpp"
#include "accelerate_control_info.hpp"
#include "brake_control_info.hpp"

#include "TCPServerBase.hpp"
#include "LCMFrequencyMeter.hpp"

// 如果不定义，则该类仅作为开关入口
// #define __ENABLE_CONTROLLER_WATCHDOG_
// __ENABLE_CONTROLLER_WATCHDOG_已定义才有效
// #define __ENABLE_LONGITUDINAL_CONTROLLER_WATCHDOG_

class ControllerWatchdog : public TCPServerBase {
    // 冗余程序的可执行文件路径和进程名
    std::string instanceDir;
    std::string instanceName;

    std::string controllerRootName;

protected:
    volatile int running_controller_count;
    // 记录instance_code和对应的pid，pid=-1则表示未运行
    std::map<int, int> instance_pids;

    bool isInstanceRunning(int instance_code) const;

#ifdef __ENABLE_CONTROLLER_WATCHDOG_
    constexpr static double MIN_LCM_FREQ = 5.0;
    constexpr static int watchdog_th_priority = 2;

    // 检测是否超时，如果超时就重启控制器
    pthread_t watchdog_th;
    static void* ThreadFunc_Watchdog(void* param);

    LCMFrequencyMeter<obu_lcm::steering_control_info> steeringFreqMeter;
#ifdef __ENABLE_LONGITUDINAL_CONTROLLER_WATCHDOG_
    LCMFrequencyMeter<obu_lcm::accelerate_control_info> accelerateFreqMeter;
    LCMFrequencyMeter<obu_lcm::brake_control_info> brakeFreqMeter;
#endif
#endif

public:
    ControllerWatchdog(const std::string &instanceDir, const std::string &instanceName, const unsigned int listenPort,
                       const std::string &steeringLcmURL = "udpm://239.255.76.63:7664?ttl=1",
                       const std::string &accelerateLcmURL = "udpm://239.255.76.63:7663?ttl=1",
                       const std::string &brakeLcmURL = "udpm://239.255.76.63:7663?ttl=1");

    virtual ~ControllerWatchdog();

    void reset();

    void resetAllFreqMeters();

    void startInstance(int instance_code);

    void killInstance(int instance_code);

    void killAllInstances();

    void restartInstance(int instance_code);

    virtual std::string onHandleTCP(const std::string tcpMessage);

};

#endif /*_CONTROLLERWATCHDOG_HPP__*/
