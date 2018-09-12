#ifndef __LCMFREQUENCYMETER_HPP__
#define __LCMFREQUENCYMETER_HPP__

#include <algorithm>
#include <sys/time.h>
#include "mytimer.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
#include <pthread.h>

template<class message_type>
class LCMFrequencyMeter {
protected:
    constexpr static int message_lcm_th_priority = 98;
    constexpr static double DOUBLE_MAX = 1e20;

    // 接收LCM数据
    struct MessageHandler {
        unsigned int pkg_count;

        struct timeval time_0, time_now;
        double last_frequency, realtime_frequency;
        double mean_frequency; // 两次平均的频率

        void handle_message(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                            const message_type *msg) {
            pkg_count++;

            if (pkg_count == 1) {
                gettimeofday(&time_0, NULL);
                return;
            }
            gettimeofday(&time_now, NULL);

            realtime_frequency = 1000000.0 / static_cast<double>(mytimer::timevalDiffInUs(&time_0, &time_now));
            mean_frequency = (pkg_count == 2) ? realtime_frequency : (last_frequency + realtime_frequency) * 0.5;

            time_0 = time_now;
            last_frequency = realtime_frequency;
        }

        MessageHandler() : pkg_count(0),
                           last_frequency(DOUBLE_MAX), realtime_frequency(DOUBLE_MAX), mean_frequency(DOUBLE_MAX) {
            gettimeofday(&time_0, NULL);
            time_now = time_0;
        }

        void reset() {
            pkg_count = 0;
            last_frequency = realtime_frequency = mean_frequency = DOUBLE_MAX;
            gettimeofday(&time_0, NULL);
            time_now = time_0;
        }

    } messageHandler;

    lcm::LCM message_lcm;
    pthread_t message_lcm_th;

    static void *ThreadFunc_Handle_Message(void *param) {
        lcm::LCM *const lcm_ptr = reinterpret_cast<lcm::LCM *const>(param);
        while (0 == lcm_ptr->handle());
        return NULL;
    }

public:
    double getRealtimeFrequency() const {
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        double frequency = 1000000.0 / static_cast<double>(mytimer::timevalDiffInUs(
                &(messageHandler.time_0), &time_now));
        return std::min(frequency, messageHandler.mean_frequency);
    }

    void reset() {
        messageHandler.reset();
    }

    LCMFrequencyMeter(const std::string &lcm_url) : message_lcm(lcm_url) {
        struct sched_param param;
        pthread_attr_t attr;
        int ret;

        /* Initialize pthread attributes (default values) */
        ret = pthread_attr_init(&attr);
        if (ret) {
            printf("init pthread attributes failed\n");
            exit(-2);
        }

        /* Set scheduler policy and priority of pthread */
        ret = pthread_attr_setschedpolicy(&attr, SCHED_RR);
        if (ret) {
            printf("pthread setschedpolicy failed\n");
            exit(-2);
        }

        //设置RT线程的优先级
        param.sched_priority = message_lcm_th_priority;
        ret = pthread_attr_setschedparam(&attr, &param);
        if (ret) {
            printf("pthread setschedparam failed\n");
            exit(-2);
        }

        /* Use scheduling parameters of attr */
        ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (ret) {
            printf("pthread setinheritsched failed\n");
            exit(-2);
        }

        /* Create a pthread with specified attributes */
        ret = pthread_create(&(message_lcm_th), &attr, &(ThreadFunc_Handle_Message), &(message_lcm));
        if (ret) {
            printf("create pthread failed\n");
            exit(-2);
        }

        message_lcm.subscribe(message_type::getTypeName(), &MessageHandler::handle_message,
                              &messageHandler);
    }

    virtual ~LCMFrequencyMeter() {
        pthread_cancel(message_lcm_th);
    }
};

#endif /*__LCMFREQUENCYMETER_HPP__*/