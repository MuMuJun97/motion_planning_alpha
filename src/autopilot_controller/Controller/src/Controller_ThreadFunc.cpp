#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include "origin_vehicle.h"
#include "Controller.hpp"
#include "draw_Controller.h"

using namespace std;

//#define __USE_PREEMPT_RT__

void *Controller::ThreadFunc_can(void *param) {
    lcm::LCM *const lcm_ptr = reinterpret_cast<lcm::LCM *const >(param);
    while (0 == lcm_ptr->handle());
    return NULL;
}

void *Controller::ThreadFunc_feedback(void *param) {
    lcm::LCM *const lcm_ptr = reinterpret_cast<lcm::LCM *const >(param);
    while (0 == lcm_ptr->handle());
    return NULL;
}

void *Controller::ThreadFunc_control_subThread(void *param) {
    Controller *const controller_ptr = reinterpret_cast<Controller *const >(param);

    while (controller_ptr->rosNodeHandle.ok()) {
        pthread_mutex_lock(&(controller_ptr->controlMutex));
        pthread_cond_wait(&(controller_ptr->controlWaitCond), &(controller_ptr->controlMutex));

        if (controller_ptr->enableLongitudinalLog)
            mytimer::getHHMMSSUS(controller_ptr->longitudinal_t0);

        controller_ptr->longitudinal_control_output(*(controller_ptr->curr_path));
        controller_ptr->last_diff_speed = controller_ptr->diff_speed;

        if (controller_ptr->enableLongitudinalLog)
            mytimer::getHHMMSSUS(controller_ptr->longitudinal_t1);

        if (controller_ptr->enableLongitudinalLog) {
            controller_ptr->longitudinalLogger.log("%d %s %s %2.2lf %2.2lf\n",
                                                   controller_ptr->ins_pkg_no, controller_ptr->longitudinal_t0,
                                                   controller_ptr->longitudinal_t1,
                                                   controller_ptr->ControlStatus.accelerate_percentage,
                                                   controller_ptr->ControlStatus.brake_percentage);
        }

        pthread_mutex_unlock(&(controller_ptr->controlMutex));
    }

    return NULL;
}

void Controller::create_pthreads() {
    pthread_mutexattr_t mutAttr;

    memset(&mutAttr, 0, sizeof(pthread_mutexattr_t));
    pthread_mutexattr_init(&mutAttr);

    pthread_mutexattr_settype(&mutAttr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&controlMutex, &mutAttr);
    pthread_cond_init(&controlWaitCond, NULL);

#ifdef __USE_PREEMPT_RT__
    /* Lock memory, must sudo */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "mlockall failed!\n" << std::endl;
        exit(-2);
    }
    struct sched_param ins_th_param, control_sub_th_param, planner_th_param;
    pthread_attr_t ins_th_attr, control_sub_th_attr, planner_th_attr;

    /* Initialize pthread attributes (default values) */
    if (pthread_attr_init(&ins_th_attr) || pthread_attr_init(&control_sub_th_attr)
            || pthread_attr_init(&planner_th_attr)) {
        std::cerr << "init pthread attributes failed!\n" << std::endl;
        exit(-2);
    }

    /* Set scheduler policy and priority of pthreads */
    if (pthread_attr_setschedpolicy(&ins_th_attr, SCHED_RR)
            || pthread_attr_setschedpolicy(&control_sub_th_attr, SCHED_RR)
            || pthread_attr_setschedpolicy(&planner_th_attr, SCHED_RR)) {
        std::cerr << "pthread setschedpolicy failed!\n" << std::endl;
        exit(-2);
    }
    ins_th_param.__sched_priority = 98;
    control_sub_th_param.__sched_priority = 98;
    planner_th_param.__sched_priority = 96;
    if (pthread_attr_setschedparam(&ins_th_attr, &ins_th_param)
            || pthread_attr_setschedparam(&control_sub_th_attr, &control_sub_th_param)
            || pthread_attr_setschedparam(&planner_th_attr, &planner_th_param)) {
        std::cerr << "pthread setschedparam failed!\n" << std::endl;
        exit(-2);
    }

    /* Use scheduling parameters of attr */
    if (pthread_attr_setinheritsched(&ins_th_attr, PTHREAD_EXPLICIT_SCHED)
            || pthread_attr_setinheritsched(&control_sub_th_attr, PTHREAD_EXPLICIT_SCHED)
            || pthread_attr_setinheritsched(&planner_th_attr, PTHREAD_EXPLICIT_SCHED)) {
        std::cerr << "pthread setinheritsched failed!\n" << std::endl;
        exit(-2);
    }
    pthread_create(&(ins_th), &ins_th_attr, &(ThreadFunc_ins), &g_lcm_ins);
    pthread_create(&(planner_th), &planner_th_attr, &(ThreadFunc_planner), &g_lcm_planner);

    pthread_create(&(control_sub_th), &control_sub_th_attr, &(ThreadFunc_control_subThread), this);

#else
    pthread_create(&(can_th), NULL, &(ThreadFunc_can), &g_lcm_can);
    pthread_create(&(feedback_th), NULL, &(ThreadFunc_feedback), &g_lcm_feedback);

    pthread_create(&(control_sub_th), NULL, &(ThreadFunc_control_subThread), this);
#endif
}

void Controller::cancel_pthreads() {
    pthread_cancel(control_sub_th);
    pthread_cancel(planner_th);

#ifndef __USE_PREEMPT_RT__
    // pthread_cancel(can_th);
    pthread_cancel(feedback_th);
#endif

    pthread_mutex_destroy(&controlMutex);
    pthread_cond_destroy(&controlWaitCond);
}
