/*-------------------------------------------------------
 * 文件名：nad_msg.h
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：引用了msg目录下的所有头文件
-------------------------------------------------------*/
#ifndef _NAD_MSG_H
#define _NAD_MSG_H


//网元内消息
#include <lcm/lcm-cpp.hpp>

//网元间消息
#include "ne_msg/ne_lcm.hpp"

//网络侧lcm消息列表
using namespace nad_lcm;

//OBU专用lcm消息列表
//using namespace obu_lcm;

//lcm消息列表
#include "nad_lcm/lane_point.hpp"
#include "nad_lcm/obu_info.hpp"
#include "nad_lcm/rsu_info.hpp"
#include "nad_lcm/light_info.hpp"
#include "nad_lcm/limspeed_info.hpp"
#include "nad_lcm/block_info.hpp"
#include "nad_lcm/map_point.hpp"
#include "nad_lcm/lane_of_route.hpp"
#include "nad_lcm/light_of_route.hpp"
#include "nad_lcm/block_of_route.hpp"
#include "nad_lcm/limspeed_of_route.hpp"
#include "nad_lcm/crossing_of_route.hpp"
#include "nad_lcm/route_element.hpp"
#include "nad_lcm/route_planning.hpp"
#include "nad_lcm/obu_config.hpp"
#include "nad_lcm/rsu_config.hpp"
#include "nad_lcm/task_config.hpp"
#include "nad_lcm/rc_rsu_login_request.hpp"
#include "nad_lcm/cr_rsu_login_respond.hpp"
#include "nad_lcm/cr_rsu_logout_notify.hpp"
#include "nad_lcm/oc_rsu_name_request.hpp"
#include "nad_lcm/co_rsu_name_respond.hpp"
#include "nad_lcm/or_obu_login_request.hpp"
#include "nad_lcm/rc_obu_login_request.hpp"
#include "nad_lcm/cr_obu_login_respond.hpp"
#include "nad_lcm/ro_obu_login_respond.hpp"
#include "nad_lcm/cr_obu_logout_notify.hpp"
#include "nad_lcm/rc_obu_logout_notify.hpp"
#include "nad_lcm/ro_obu_logout_notify.hpp"
#include "nad_lcm/uo_route_request.hpp"
#include "nad_lcm/or_route_request.hpp"
#include "nad_lcm/rc_route_request.hpp"
#include "nad_lcm/cr_route_respond.hpp"
#include "nad_lcm/ro_route_respond.hpp"
#include "nad_lcm/ou_route_respond.hpp"
#include "nad_lcm/uo_start_auto_request.hpp"
#include "nad_lcm/or_start_auto_request.hpp"
#include "nad_lcm/rc_start_auto_request.hpp"
#include "nad_lcm/cr_start_auto_respond.hpp"
#include "nad_lcm/ro_start_auto_respond.hpp"
#include "nad_lcm/ou_start_auto_respond.hpp"
#include "nad_lcm/uo_stop_auto_request.hpp"
#include "nad_lcm/or_stop_auto_notify.hpp"
#include "nad_lcm/rc_stop_auto_notify.hpp"
#include "nad_lcm/cu_stop_auto_notify.hpp"
#include "nad_lcm/ou_stop_auto_respond.hpp"
#include "nad_lcm/er_ets_report.hpp"
#include "nad_lcm/re_ets_report.hpp"
#include "nad_lcm/or_info_report.hpp"
#include "nad_lcm/rc_info_report.hpp"
#include "nad_lcm/cu_info_report.hpp"
#include "nad_lcm/cr_info_report.hpp"
#include "nad_lcm/ro_info_report.hpp"
#include "nad_lcm/ou_info_report.hpp"
#include "nad_lcm/obu_control_event.hpp"
#include "nad_lcm/or_event_notify.hpp"
#include "nad_lcm/ou_event_notify.hpp"
#include "nad_lcm/or_change_lane_request.hpp"
#include "nad_lcm/ro_change_lane_notify.hpp"
#include "nad_lcm/ou_change_lane_notify.hpp"
#include "nad_lcm/ro_change_speed_notify.hpp"
#include "nad_lcm/ou_change_speed_notify.hpp"
#include "nad_lcm/uc_oct_login_request.hpp"
#include "nad_lcm/cu_oct_login_respond.hpp"
#include "nad_lcm/uc_config_request.hpp"
#include "nad_lcm/cu_config_respond.hpp"
#include "nad_lcm/uc_exec_task_request.hpp"
#include "nad_lcm/cu_exec_task_respond.hpp"
#include "nad_lcm/csu_task_func_request.hpp"
#include "nad_lcm/csu_task_func_respond.hpp"
#include "nad_lcm/uc_set_ets_request.hpp"
#include "nad_lcm/cr_set_ets_request.hpp"
#include "nad_lcm/cu_set_ets_respond.hpp"
#include "nad_lcm/rc_set_ets_respond.hpp"
#include "nad_lcm/cu_log_report.hpp"
#include "nad_lcm/cu_alarm_report.hpp"
#include "nad_lcm/ou_log_report.hpp"
#include "nad_lcm/ou_alarm_report.hpp"

//OBU专用lcm消息列表
#include "obu_lcm/accelerate_control_info.hpp"
#include "obu_lcm/accelerate_feedback_info.hpp"
#include "obu_lcm/brake_control_info.hpp"
#include "obu_lcm/brake_feedback_info.hpp"
#include "obu_lcm/CAN_status.hpp"
#include "obu_lcm/CAN_value.hpp"
#include "obu_lcm/gears_control_info.hpp"
#include "obu_lcm/gears_feedback_info.hpp"
#include "obu_lcm/gps_info.hpp"
#include "obu_lcm/ins_info.hpp"
#include "obu_lcm/map_line.hpp"
#include "obu_lcm/map_points.hpp"
#include "obu_lcm/nav_control_points.hpp"
#include "obu_lcm/nav_points.hpp"
#include "obu_lcm/obstacle_list.hpp"
#include "obu_lcm/obstacle_t.hpp"
#include "obu_lcm/nav_control_points.hpp"
#include "obu_lcm/obu_map_info.hpp"
#include "obu_lcm/patch_grid.hpp"
#include "obu_lcm/patch_t.hpp"
#include "obu_lcm/point_t.hpp"
#include "obu_lcm/rect_t.hpp"
#include "obu_lcm/steering_control_info.hpp"
#include "obu_lcm/steering_feedback_info.hpp"
#include "obu_lcm/mo_planning_request.hpp"
#include "obu_lcm/obu_behavior_info.hpp"
#include "obu_lcm/esr_data_t.hpp"
#include "obu_lcm/esr_data_list.hpp"

//定时器
#include "timer/nad_timer_list.h"

//SESSION(控制块)
#include "session/nad_session_list.h"

//消息公共的url

#ifdef _NAD_CSU_
#define LCM_URL "udpm://239.255.76.61:7667?ttl=3"
#endif
#ifdef _NAD_RSU_
#define LCM_URL "udpm://239.255.76.62:7667?ttl=3"
#endif
#ifdef _NAD_OBU_
#define LCM_URL "udpm://239.255.76.63:7667?ttl=3"
#endif

//调试阶段使用的url
/*#ifdef _NAD_CSU_
#define LCM_URL "udpm://239.255.76.61:7667?ttl=3"
#endif
#ifdef _NAD_RSU_
#define LCM_URL "udpm://239.255.76.61:7667?ttl=3"
#endif
#ifdef _NAD_OBU_
#define LCM_URL "udpm://239.255.76.61:7667?ttl=3"
#endif
*/
//nemsg.header.type的宏定义，按照系统基本流程来编号
#define RC_RSU_LOGIN_REQUEST        1
#define CR_RSU_LOGIN_RESPOND        2
#define RC_INFO_REPORT              3
#define CR_INFO_REPORT              4
#define CR_RSU_LOGOUT_NOTIFY        5
#define OC_RSU_NAME_REQUEST         6
#define CO_RSU_NAME_RESPOND	        7
#define OR_OBU_LOGIN_REQUEST	    8
#define RC_OBU_LOGIN_REQUEST        9
#define CR_OBU_LOGIN_RESPOND        10
#define RO_OBU_LOGIN_RESPOND        11
#define OR_INFO_REPORT              12
#define RO_INFO_REPORT		        13
#define CR_OBU_LOGOUT_NOTIFY        14
#define RC_OBU_LOGOUT_NOTIFY        15
#define RO_OBU_LOGOUT_NOTIFY        16
#define UO_ROUTE_REQUEST            17
#define OR_ROUTE_REQUEST            18
#define RC_ROUTE_REQUEST            19
#define CR_ROUTE_RESPOND            20
#define RO_ROUTE_RESPOND            21
#define OU_ROUTE_RESPOND            22
#define UO_START_AUTO_REQUEST       23
#define OR_START_AUTO_REQUEST       24
#define RC_START_AUTO_REQUEST       25
#define CR_START_AUTO_RESPOND       26
#define RO_START_AUTO_RESPOND       27
#define OU_START_AUTO_RESPOND       28
#define OU_INFO_REPORT              29
#define ER_ETS_REPORT               30
#define RE_ETS_REPORT               31
#define UC_OCT_LOGIN_REQUEST        32
#define CU_OCT_LOGIN_RESPOND        33
#define CU_INFO_REPORT              34
#define UC_SET_ETS_REQUEST          35
#define CR_SET_ETS_REQUEST          36
#define RC_SET_ETS_RESPOND          37
#define CU_SET_ETS_RESPOND          38
#define UC_CONFIG_REQUEST           39
#define CU_CONFIG_RESPOND           40
#define UC_EXEC_TASK_REQUEST        41
#define CU_EXEC_TASK_RESPOND        42
#define CSU_TASK_FUNC_REQUEST       43
#define CSU_TASK_FUNC_RESPOND       44
#define UO_STOP_AUTO_REQUEST        45
#define OR_STOP_AUTO_NOTIFY         46
#define RC_STOP_AUTO_NOTIFY         47
#define CU_STOP_AUTO_NOTIFY         48
#define OU_STOP_AUTO_RESPOND        49
#define OBU_CONTROL_EVENT           50
#define OR_EVENT_NOTIFY             51
#define OU_EVENT_NOTIFY             52
#define CU_LOG_REPORT               53
#define CU_ALARM_REPORT             54
#define OU_LOG_REPORT               55
#define OU_ALARM_REPORT             56
#define OR_CHANGE_LANE_REQUEST      57
#define RO_CHANGE_LANE_NOTIFY       58
#define OU_CHANGE_LANE_NOTIFY       59
#define RO_CHANGE_SPEED_NOTIFY      60
#define OU_CHANGE_SPEED_NOTIFY      61
//网元间消息写日志
void log_ne_msg(int need_send, const ne_msg_base_t* ne_msg);
#endif
