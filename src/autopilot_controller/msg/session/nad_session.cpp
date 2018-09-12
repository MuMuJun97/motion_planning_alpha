/*-------------------------------------------------------
 * 文件名：nad_session.cpp
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：SESSION(控制块)的基类
-------------------------------------------------------*/


//头文件
#include "nad_session.h"


//构造SESSION(控制块)
nad_session::nad_session(string name)
{
    this->name = name;
}

//析构函数
nad_session::~nad_session()
{
}

//处理定时器
void nad_session::handle_timer()
{
}

//注册消息到lcm
void nad_session::reg_msg_to_lcm(NE_LCM *lcm)
{
}
