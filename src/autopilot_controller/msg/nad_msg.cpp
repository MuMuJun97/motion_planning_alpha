/*-------------------------------------------------------
 * 文件名：nad_msg.cpp
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：引用了msg目录下的所有头文件
-------------------------------------------------------*/


#include "nad_msg.h"


//网元间消息写日志
//格式为：
//  send: rsu->csu: RC_RSU_LOGIN_REQUEST=1: rsu=rsu_1
//  recv: rsu<-obu: OR_INFO_REPORT=12: obu=车1 lon=10.1 lat=10.2
void log_ne_msg(int need_send, const ne_msg_base_t* ne_msg)
{
    char dirt[16] = "";
    char name[32] = "";
    char para[128] = "";
    
    //消息指针
    cr_rsu_login_respond msg2;
    rc_info_report msg3;
    oc_rsu_name_request msg6;
    co_rsu_name_respond msg7;
    cr_obu_login_respond msg10;
    ro_obu_login_respond msg11;
    or_info_report msg12;
    ro_info_report msg13;
    or_route_request msg18;
    rc_route_request msg19;
    cr_route_respond msg20;
    ro_route_respond msg21;
    or_start_auto_request msg24;
    rc_start_auto_request msg25;
    cr_start_auto_respond msg26;
    ro_start_auto_respond msg27;
    uc_oct_login_request msg32;
    cu_oct_login_respond msg33;
    cr_set_ets_request msg36;
    rc_set_ets_respond msg37;
    or_stop_auto_notify msg46;
    rc_stop_auto_notify msg47;
    or_change_lane_request msg57;
    ro_change_lane_notify msg58;
    
    //获得消息类型和参数
    switch (ne_msg->header.type)
    {
        case RC_RSU_LOGIN_REQUEST   : //1
            strcpy(name, "RC_RSU_LOGIN_REQUEST");
            break;
        case CR_RSU_LOGIN_RESPOND   : //2
            strcpy(name, "CR_RSU_LOGIN_RESPOND");
            ((ne_msg_t<cr_rsu_login_respond> *)ne_msg)->decode_body(msg2);
            sprintf(para, "ret=%d", msg2.retcode);
            break;
        case RC_INFO_REPORT         : //3
            strcpy(name, "RC_INFO_REPORT");
            ((ne_msg_t<rc_info_report> *)ne_msg)->decode_body(msg3);
            sprintf(para, "inline_obu=%d, light=%d, limspeed=%d, block=%d", 
                msg3.rsu.num_of_obu, msg3.num_of_light, msg3.num_of_limspeed, msg3.num_of_block);
            break;
        case CR_INFO_REPORT         : //4
            strcpy(name, "CR_INFO_REPORT");
            break;
        case CR_RSU_LOGOUT_NOTIFY   : //5
            strcpy(name, "CR_RSU_LOGOUT_NOTIFY");
            break;
        case OC_RSU_NAME_REQUEST    : //6
            strcpy(name, "OC_RSU_NAME_REQUEST");
            ((ne_msg_t<oc_rsu_name_request> *)ne_msg)->decode_body(msg6);
            sprintf(para, "obu=%s, lon=%f, lat=%f", msg6.obu_name.c_str(), msg6.obu_lon, msg6.obu_lat);
            break;
        case CO_RSU_NAME_RESPOND	: //7
            strcpy(name, "CO_RSU_NAME_RESPOND");
            ((ne_msg_t<co_rsu_name_respond> *)ne_msg)->decode_body(msg7);
            sprintf(para, "rsu=%s", msg7.rsu_name.c_str());
            break;
        case OR_OBU_LOGIN_REQUEST	: //8
            strcpy(name, "OR_OBU_LOGIN_REQUEST");
            break;
        case RC_OBU_LOGIN_REQUEST   : //9
            strcpy(name, "RC_OBU_LOGIN_REQUEST");
            break;
        case CR_OBU_LOGIN_RESPOND   : //10
            strcpy(name, "CR_OBU_LOGIN_RESPOND");
            ((ne_msg_t<cr_obu_login_respond> *)ne_msg)->decode_body(msg10);
            sprintf(para, "ret=%d", msg10.retcode);
            break;
        case RO_OBU_LOGIN_RESPOND   : //11
            strcpy(name, "RO_OBU_LOGIN_RESPOND");
            ((ne_msg_t<ro_obu_login_respond> *)ne_msg)->decode_body(msg11);
            sprintf(para, "ret=%d", msg11.retcode);
            break;
        case OR_INFO_REPORT         : //12
            strcpy(name, "OR_INFO_REPORT");
            ((ne_msg_t<or_info_report> *)ne_msg)->decode_body(msg12);
            sprintf(para, "lon=%f, lat=%f, yaw=%f, lane=%lld, off=%f, speed=%f, steer=%f, gear=%d", 
                msg12.obu.cur_lon, msg12.obu.cur_lat, msg12.obu.cur_yaw, msg12.obu.cur_point.lane_id, 
                msg12.obu.cur_point.offset, msg12.obu.cur_speed, msg12.obu.steering_angle, msg12.obu.cur_gears);
            break;
        case RO_INFO_REPORT		    : //13
            strcpy(name, "RO_INFO_REPORT");
            ((ne_msg_t<ro_info_report> *)ne_msg)->decode_body(msg13);
            sprintf(para, "light=%d, limspeed=%d, block=%d", 
                msg13.num_of_light, msg13.num_of_limspeed, msg13.num_of_block);
            break;
        case CR_OBU_LOGOUT_NOTIFY   : //14
            strcpy(name, "CR_OBU_LOGOUT_NOTIFY");
            break;
        case RC_OBU_LOGOUT_NOTIFY   : //15
            strcpy(name, "RC_OBU_LOGOUT_NOTIFY");
            break;
        case RO_OBU_LOGOUT_NOTIFY   : //16
            strcpy(name, "RO_OBU_LOGOUT_NOTIFY");
            break;
        case UO_ROUTE_REQUEST       : //17
            strcpy(name, "UO_ROUTE_REQUEST");
            break;
        case OR_ROUTE_REQUEST       : //18
            strcpy(name, "OR_ROUTE_REQUEST");
            ((ne_msg_t<or_route_request> *)ne_msg)->decode_body(msg18);
            sprintf(para, "obu=%s, start=%f/%f, end=%f/%f/%s, reason=%d", 
                msg18.obu_name.c_str(), msg18.starting_lon, msg18.starting_lat,
                msg18.ending_lon, msg18.ending_lat, msg18.destination.c_str(), msg18.route_reason);
            break;
        case RC_ROUTE_REQUEST       : //19
            strcpy(name, "RC_ROUTE_REQUEST");
            ((ne_msg_t<rc_route_request> *)ne_msg)->decode_body(msg19);
            sprintf(para, "obu=%s, start=%f/%f, end=%f/%f/%s, reason=%d", 
                msg19.obu_name.c_str(), msg19.starting_lon, msg19.starting_lat,
                msg19.ending_lon, msg19.ending_lat, msg19.destination.c_str(), msg19.route_reason);
            break;
        case CR_ROUTE_RESPOND       : //20
            strcpy(name, "CR_ROUTE_RESPOND");
            ((ne_msg_t<cr_route_respond> *)ne_msg)->decode_body(msg20);
            sprintf(para, "ret=%d", msg20.retcode);
            break;
        case RO_ROUTE_RESPOND       : //21
            strcpy(name, "RO_ROUTE_RESPOND");
            ((ne_msg_t<ro_route_respond> *)ne_msg)->decode_body(msg21);
            sprintf(para, "ret=%d", msg21.retcode);
            break;
        case OU_ROUTE_RESPOND       : //22
            strcpy(name, "OU_ROUTE_RESPOND");
            break;
        case UO_START_AUTO_REQUEST  : //23
            strcpy(name, "UO_START_AUTO_REQUEST");
            break;
        case OR_START_AUTO_REQUEST  : //24
            strcpy(name, "OR_START_AUTO_REQUEST");
            ((ne_msg_t<or_start_auto_request> *)ne_msg)->decode_body(msg24);
            sprintf(para, "time=%lld, reason=%d", msg24.time_stamp, msg24.start_reason);
            break;
        case RC_START_AUTO_REQUEST  : //25
            strcpy(name, "RC_START_AUTO_REQUEST");
            ((ne_msg_t<rc_start_auto_request> *)ne_msg)->decode_body(msg25);
            sprintf(para, "time=%lld, reason=%d", msg25.time_stamp, msg25.start_reason);
            break;
        case CR_START_AUTO_RESPOND  : //26
            strcpy(name, "CR_START_AUTO_RESPOND");
            ((ne_msg_t<cr_start_auto_respond> *)ne_msg)->decode_body(msg26);
            sprintf(para, "ret=%d, reason=%d", msg26.retcode, msg26.start_reason);
            break;
        case RO_START_AUTO_RESPOND  : //27
            strcpy(name, "RO_START_AUTO_RESPOND");
            ((ne_msg_t<ro_start_auto_respond> *)ne_msg)->decode_body(msg27);
            sprintf(para, "ret=%d, reason=%d", msg27.retcode, msg27.start_reason);
            break;
        case OU_START_AUTO_RESPOND  : //28
            strcpy(name, "OU_START_AUTO_RESPOND");
            break;
        case OU_INFO_REPORT         : //29
            strcpy(name, "OU_INFO_REPORT");
            break;
        case ER_ETS_REPORT          : //30
            strcpy(name, "ER_ETS_REPORT");
            break;
        case RE_ETS_REPORT          : //31
            strcpy(name, "RE_ETS_REPORT");
            break;
        case UC_OCT_LOGIN_REQUEST   : //32
            strcpy(name, "UC_OCT_LOGIN_REQUEST");
            ((ne_msg_t<uc_oct_login_request> *)ne_msg)->decode_body(msg32);
            sprintf(para, "usr=%s, pwd=%s", msg32.csu_user.c_str(), msg32.csu_password.c_str());
            break;
        case CU_OCT_LOGIN_RESPOND   : //33
            strcpy(name, "CU_OCT_LOGIN_RESPOND");
            ((ne_msg_t<cu_oct_login_respond> *)ne_msg)->decode_body(msg33);
            sprintf(para, "ret=%d", msg33.retcode);
            break;
        case CU_INFO_REPORT         : //34
            strcpy(name, "CU_INFO_REPORT");
            break;
        case UC_SET_ETS_REQUEST     : //35
            strcpy(name, "UC_SET_ETS_REQUEST");
            break;
        case CR_SET_ETS_REQUEST     : //36
            strcpy(name, "CR_SET_ETS_REQUEST");
            ((ne_msg_t<cr_set_ets_request> *)ne_msg)->decode_body(msg36);
            sprintf(para, "id=%s, type=%d, set=%s, val=%d, reason=%d", 
                msg36.ets_id.c_str(), msg36.ets_type, msg36.ets_setting.c_str(), msg36.ets_value, msg36.reason);
           break;
        case RC_SET_ETS_RESPOND     : //37
            strcpy(name, "RC_SET_ETS_RESPOND");
            ((ne_msg_t<rc_set_ets_respond> *)ne_msg)->decode_body(msg37);
            sprintf(para, "id=%s, ret=%d, err=%s, reason=%d", 
                msg37.ets_id.c_str(), msg37.retcode, msg37.err_desc.c_str(), msg37.reason);
            break;
        case CU_SET_ETS_RESPOND     : //38
            strcpy(name, "CU_SET_ETS_RESPOND");
            break;
        case UC_CONFIG_REQUEST      : //39
            strcpy(name, "UC_CONFIG_REQUEST");
            break;
        case CU_CONFIG_RESPOND      : //40
            strcpy(name, "CU_CONFIG_RESPOND");
            break;
        case UC_EXEC_TASK_REQUEST   : //41
            strcpy(name, "UC_EXEC_TASK_REQUEST");
            break;
        case CU_EXEC_TASK_RESPOND   : //42
            strcpy(name, "CU_EXEC_TASK_RESPOND");
            break;
        case CSU_TASK_FUNC_REQUEST  : //43
            strcpy(name, "CSU_TASK_FUNC_REQUEST");
            break;
        case CSU_TASK_FUNC_RESPOND  : //44
            strcpy(name, "CSU_TASK_FUNC_RESPOND");
            break;
        case UO_STOP_AUTO_REQUEST   : //45
            strcpy(name, "UO_STOP_AUTO_REQUEST");
            break;
        case OR_STOP_AUTO_NOTIFY    : //46
            strcpy(name, "OR_STOP_AUTO_NOTIFY");
            ((ne_msg_t<or_stop_auto_notify> *)ne_msg)->decode_body(msg46);
            sprintf(para, "reason=%d", msg46.stop_reason);
            break;
        case RC_STOP_AUTO_NOTIFY    : //47
            strcpy(name, "RC_STOP_AUTO_NOTIFY");
            ((ne_msg_t<rc_stop_auto_notify> *)ne_msg)->decode_body(msg47);
            sprintf(para, "reason=%d", msg47.stop_reason);
            break;
        case CU_STOP_AUTO_NOTIFY    : //48
            strcpy(name, "CU_STOP_AUTO_NOTIFY");
            break;
        case OU_STOP_AUTO_RESPOND   : //49
            strcpy(name, "OU_STOP_AUTO_RESPOND");
            break;
        case OBU_CONTROL_EVENT      : //50
            strcpy(name, "OBU_CONTROL_EVENT");
            break;
        case OR_EVENT_NOTIFY        : //51
            strcpy(name, "OR_EVENT_NOTIFY");
            break;
        case OU_EVENT_NOTIFY        : //52
            strcpy(name, "OU_EVENT_NOTIFY");
            break;
        case CU_LOG_REPORT          : //53
            strcpy(name, "CU_LOG_REPORT");
            break;
        case CU_ALARM_REPORT        : //54
            strcpy(name, "CU_ALARM_REPORT");
            break;
        case OU_LOG_REPORT          : //55
            strcpy(name, "OU_LOG_REPORT");
            break;
        case OU_ALARM_REPORT        : //56
            strcpy(name, "OU_ALARM_REPORT");
            break;
        case OR_CHANGE_LANE_REQUEST : //57
            strcpy(name, "OR_CHANGE_LANE_REQUEST");
            ((ne_msg_t<or_change_lane_request> *)ne_msg)->decode_body(msg57);
            sprintf(para, "active=%lld/%f, end=%lld/%f, dirt=%d, level=%d, resaon=%d", 
                msg57.active_point.lane_id, msg57.active_point.offset, msg57.end_point.lane_id, 
                msg57.end_point.offset, msg57.direction, msg57.level, msg57.reason);
            break;
        case RO_CHANGE_LANE_NOTIFY  : //58
            strcpy(name, "RO_CHANGE_LANE_NOTIFY");
            ((ne_msg_t<ro_change_lane_notify> *)ne_msg)->decode_body(msg58);
            sprintf(para, "ret=%d, active=%lld/%f, end=%lld/%f, dirt=%d, level=%d, resaon=%d", 
                msg58.retcode, msg58.active_point.lane_id, msg58.active_point.offset, msg58.end_point.lane_id, 
                msg58.end_point.offset, msg58.direction, msg57.level, msg58.reason);
            break;
        case OU_CHANGE_LANE_NOTIFY  : //59
            strcpy(name, "OU_CHANGE_LANE_NOTIFY");
            break;
        case RO_CHANGE_SPEED_NOTIFY : //60
            strcpy(name, "RO_CHANGE_SPEED_NOTIFY");
            break;
        case OU_CHANGE_SPEED_NOTIFY : //61
            strcpy(name, "OU_CHANGE_SPEED_NOTIFY");
            break;
        default:
            strcpy(name, "UNKNOWN_MSG");
    }
    
    //判断消息类型
    if (need_send)
    {
        if (name[2] != '_') {
            strcpy(dirt, "send");
        } else if ((name[0] == 'R') && (name[1] == 'O')) {
            strcpy(dirt, "send: rsu->obu");
        } else if ((name[0] == 'O') && (name[1] == 'R')) {
            strcpy(dirt, "send: obu->rsu");
        } else if ((name[0] == 'C') && (name[1] == 'R')) {
            strcpy(dirt, "send: csu->rsu");
        } else if ((name[0] == 'R') && (name[1] == 'C')) {
            strcpy(dirt, "send: rsu->csu");
        } else if ((name[0] == 'O') && (name[1] == 'C')) {
            strcpy(dirt, "send: obu->csu");
        } else if ((name[0] == 'C') && (name[1] == 'O')) {
            strcpy(dirt, "send: csu->obu");
        } else {
            strcpy(dirt, "send");
        }
    }
    else
    {
        if (name[2] != '_') {
            strcpy(dirt, "recv");
        } else if ((name[0] == 'R') && (name[1] == 'O')) {
            strcpy(dirt, "recv: obu<-rsu");
        } else if ((name[0] == 'O') && (name[1] == 'R')) {
            strcpy(dirt, "recv: rsu<-obu");
        } else if ((name[0] == 'C') && (name[1] == 'R')) {
            strcpy(dirt, "recv: rsu<-csu");
        } else if ((name[0] == 'R') && (name[1] == 'C')) {
            strcpy(dirt, "recv: csu<-rsu");
        } else if ((name[0] == 'O') && (name[1] == 'C')) {
            strcpy(dirt, "recv: csu<-obu");
        } else if ((name[0] == 'C') && (name[1] == 'O')) {
            strcpy(dirt, "recv: obu<-csu");
        } else {
            strcpy(dirt, "recv");
        }
    }
    
    //打印日志
    LOG(WARNING) << dirt << ": " << name << "=" << ne_msg->header.type << ": " << para;
    
    //追加一些附加信息
    
}

