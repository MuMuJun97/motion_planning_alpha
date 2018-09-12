/*-------------------------------------------------------
 * 文件名：nad_session_list.h
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：SESSION(控制块)列表
-------------------------------------------------------*/
#ifndef _NAD_SESSION_LIST_H
#define _NAD_SESSION_LIST_H


#include "nad_session.h"


//SESSION(控制块)管理器的基类
class nad_session_list
{
protected:
    //SESSION(控制块)列表
    map<string, nad_session*> session_map;

    //最大session数量
    int max_size;

    //定时器
    nad_timer *timer;

public:
    //构造析构函数，参数(最大session数&0=无限, 定时器间隔&0=不起定时器)
    nad_session_list(int max_size, int64 interval_ms);
    virtual ~nad_session_list();

    //获得session数量，禁止重载
    int size();

    //新增SESSION(控制块)，session->name不能重复!
    virtual int add_session(nad_session *session);

    //查询SESSION(控制块)，name不能重复!
    virtual nad_session *find_session(string name);

    //删除SESSION(控制块)，成功返回ERT_OK
    virtual int delete_session(string name);

    //删除所有SESSION(控制块)
    virtual int clear_session();

    //处理消息，如果收到的不是ne_msg_t必须重载，否则内部强转会coredump
    virtual int handle_msg(const void *nemsg);

    //处理定时器，通常不用重载
    virtual void handle_timer();

    //注册消息到lcm中的函数
    virtual void reg_msg_to_lcm(NE_LCM *lcm);
};


#endif
