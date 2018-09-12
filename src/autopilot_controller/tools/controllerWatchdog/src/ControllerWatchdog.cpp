#include "ControllerWatchdog.hpp"
#include "MySystemAPI.hpp"

#include <iostream>
#include <sstream>
#include <cstdio>

#include <pthread.h>
#include <unistd.h>
#include <limits.h>
#include <sched.h>
#include <sys/mman.h>

using namespace std;

#ifdef __ENABLE_CONTROLLER_WATCHDOG_
void* ControllerWatchdog::ThreadFunc_Watchdog(void* param) {
    ControllerWatchdog* const watchdog_ptr = reinterpret_cast<ControllerWatchdog* const>(param);
    int last_running_controller_count = 0;
    constexpr unsigned int start_time_consume = 1500000;

    while (true) {
        usleep(40000);
        if (watchdog_ptr->running_controller_count <= 0) {
            last_running_controller_count = watchdog_ptr->running_controller_count;
            continue;
        }
        usleep(160000);
        if (last_running_controller_count == 0)
            usleep(start_time_consume);

        double steeringFreq = watchdog_ptr->steeringFreqMeter.getRealtimeFrequency();

#ifdef __ENABLE_LONGITUDINAL_CONTROLLER_WATCHDOG_
        double accelerateFreq = watchdog_ptr->accelerateFreqMeter.getRealtimeFrequency();
        double brakeFreq = watchdog_ptr->brakeFreqMeter.getRealtimeFrequency();

        if (steeringFreq < MIN_LCM_FREQ || accelerateFreq < MIN_LCM_FREQ || brakeFreq < MIN_LCM_FREQ)
#else
        if (steeringFreq < MIN_LCM_FREQ)
#endif
        {
            if (watchdog_ptr->running_controller_count > 0) {
                cerr << "ControllerWatchdog::ThreadFunc_Watchdog: "
                     << "steeringFreq=" << steeringFreq << ", "
#ifdef __ENABLE_LONGITUDINAL_CONTROLLER_WATCHDOG_
                     << "accelerateFreq=" << accelerateFreq << ","
                     << "brakeFreq=" << brakeFreq << ", "
#endif
                     << "restart controller" << endl;

                for (map<int, int>::iterator it = watchdog_ptr->instance_pids.begin();
                        it != watchdog_ptr->instance_pids.end(); ++it) {
                    if (it->second != -1)
                        watchdog_ptr->restartInstance(it->first);
                }

                usleep(start_time_consume);
            }
        }

        last_running_controller_count = watchdog_ptr->running_controller_count;
    }

    return NULL;
}
#endif

bool ControllerWatchdog::isInstanceRunning(int instance_code) const {
    return instance_pids.count(instance_code) > 0 && instance_pids.at(instance_code) != -1;
}

ControllerWatchdog::ControllerWatchdog(const std::string &instanceDir,
                                       const std::string &instanceName,
                                       const unsigned int listenPort,
                                       const std::string &steeringLcmURL,
                                       const std::string &accelerateLcmURL,
                                       const std::string &brakeLcmURL) :
        instanceDir(instanceDir), instanceName(instanceName),
        controllerRootName(""), running_controller_count(0),
#ifdef __ENABLE_CONTROLLER_WATCHDOG_
steeringFreqMeter(steeringLcmURL),
#ifdef __ENABLE_LONGITUDINAL_CONTROLLER_WATCHDOG_
accelerateFreqMeter(accelerateLcmURL), brakeFreqMeter(brakeLcmURL),
#endif
#endif
        TCPServerBase(listenPort) {
#ifdef __ENABLE_CONTROLLER_WATCHDOG_
    struct sched_param param;
    pthread_attr_t attr;
    int ret;

    /* Lock memory */
    //将程序的整个虚拟内存地址锁定在内存中
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        //提示“Cannot Allocate Memory”，可能是可执行文件的权限不够
        printf("mlockall failed: %m\n");
        exit(-2);
    }

    /* Initialize pthread attributes (default values) */
    //绑定用于初始化RT线程的参数
    ret = pthread_attr_init(&attr);
    if (ret) {
        printf("init pthread attributes failed\n");
        exit(-2);
    }

    /* Set scheduler policy and priority of pthread */
    //设置RT线程的调度策略
    //SCHED_FIFO：先进先出，优先服务
    //SCHED_RR：轮询调度
    //SCHED_DEADLINE：全局最早Deadline优先
    ret = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        exit(-2);
    }

    //设置RT线程的优先级
    param.sched_priority = watchdog_th_priority;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
        printf("pthread setschedparam failed\n");
        exit(-2);
    }

    /* Use scheduling parameters of attr */
    //设置RT线程的调度策略和优先级必须是上述显式定义的数值，而非从创建者继承
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        printf("pthread setinheritsched failed\n");
        exit(-2);
    }

    /* Create a pthread with specified attributes */
    //根据上述显式参数，创建RT线程
    ret = pthread_create(&(watchdog_th), &attr, &(ThreadFunc_Watchdog), this);
    if (ret) {
        printf("create pthread failed\n");
        exit(-2);
    }
#endif
}

ControllerWatchdog::~ControllerWatchdog() {
#ifdef __ENABLE_CONTROLLER_WATCHDOG_
    pthread_cancel(watchdog_th);
#endif

    killAllInstances();
}

void ControllerWatchdog::reset() {
    killAllInstances();

    controllerRootName = "";

    running_controller_count = 0;
    instance_pids.clear();

    resetAllFreqMeters();
}

void ControllerWatchdog::resetAllFreqMeters() {
#ifdef __ENABLE_CONTROLLER_WATCHDOG_
    steeringFreqMeter.reset();

#ifdef __ENABLE_LONGITUDINAL_CONTROLLER_WATCHDOG_
    accelerateFreqMeter.reset();
    brakeFreqMeter.reset();
#endif

#endif
}

void ControllerWatchdog::startInstance(int instance_code) {
    if (isInstanceRunning(instance_code))
        return;

    const string instance_code_str = to_string(instance_code);
    const string controllerName = controllerRootName + instance_code_str;
    const char *const const_argv[] = {instanceName.c_str(),
                                      "-c", instance_code_str.c_str(),
                                      "-n", controllerName.c_str(),
                                      NULL
    };
    char *const *argv = const_cast<char *const *>(const_argv);
    instance_pids[instance_code] = MySystemAPI::exec(instanceDir + instanceName, argv);

    running_controller_count++;
}

void ControllerWatchdog::killInstance(int instance_code) {
    if (!isInstanceRunning(instance_code))
        return;

    system(("kill -9 " + to_string(instance_pids[instance_code])).c_str());
    instance_pids[instance_code] = -1;
    running_controller_count--;
}

void ControllerWatchdog::killAllInstances() {
    system(("killall -9 " + instanceName).c_str());
}

void ControllerWatchdog::restartInstance(int instance_code) {
    killInstance(instance_code);
    startInstance(instance_code);
}

std::string ControllerWatchdog::onHandleTCP(const std::string tcpMessage) {
    cout << tcpMessage << endl;

    istringstream msgss(tcpMessage);
    // could be reset/start/stop/restart
    string msgOperation, msgControllerRootName;
    int firstInstanceCode, lastInstanceCode;

    if (tcpMessage.compare("reset ControllerWatchdog") == 0) {
        reset();
    } else if (msgss >> msgOperation >> msgControllerRootName >> firstInstanceCode >> lastInstanceCode) {
        if (msgOperation.compare("start") == 0) {
            controllerRootName = msgControllerRootName;
            for (int i = firstInstanceCode; i <= lastInstanceCode; ++i)
                startInstance(i);
        } else if (msgOperation.compare("stop") == 0) {
            if (controllerRootName.compare(msgControllerRootName) == 0) {
                for (int i = firstInstanceCode; i <= lastInstanceCode; ++i)
                    killInstance(i);
            } else {
                cerr << "ControllerWatchdog::onHandleTCP: controllerRootName in stop message \""
                     << tcpMessage << "\" is wrong" << endl;
            }

        } else if (msgOperation.compare("restart") == 0) {
            if (controllerRootName.empty())
                controllerRootName = msgControllerRootName;

            if (controllerRootName.compare(msgControllerRootName) == 0) {
                for (int i = firstInstanceCode; i <= lastInstanceCode; ++i)
                    restartInstance(i);
            } else {
                cerr << "ControllerWatchdog::onHandleTCP: controllerRootName in restart message \""
                     << tcpMessage << "\" is wrong" << endl;
            }
        } else {
            cerr << "ControllerWatchdog::onHandleTCP: invalid operation in \"" << tcpMessage << "\"" << endl;
        }
    } else {
        cerr << "ControllerWatchdog::onHandleTCP: invalid message \"" << tcpMessage << "\"" << endl;
    }

    return string("");
}
