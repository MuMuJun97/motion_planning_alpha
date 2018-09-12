/*-------------------------------------------------------
 * 文件名：nad_session.h
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：SESSION(控制块)的基类
-------------------------------------------------------*/
#ifndef _NAD_SESSION_H
#define _NAD_SESSION_H

#include "../timer/nad_timer_list.h"
#include "../ne_msg/ne_lcm.hpp"

//SESSION(控制块)的基类
class nad_session
{
public:
    //对象名称
    string name;

public:
    //构造析构函数
    nad_session(string name);
    virtual ~nad_session();

    //处理消息
    virtual int handle_msg(const void *nemsg) = 0;

    //处理定时器
    virtual void handle_timer();

    //注册消息到lcm
    virtual void reg_msg_to_lcm(NE_LCM *lcm);
};


#endif
