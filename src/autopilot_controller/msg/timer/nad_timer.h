/*-------------------------------------------------------
 * 文件名：nad_timer.h
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：定时器的基类
-------------------------------------------------------*/
#ifndef _nad_timer_H
#define _nad_timer_H


//引用model头文件
#include "../../model/nad_model.h"


//定时器的基类
class nad_timer
{
public:
    //定时器时间间隔
    int64 interval_ms;

    //上次handle的时间
    int64 last_ms;

public:
    //构造析构函数
    nad_timer(int64 interval_ms);
    virtual ~nad_timer();

    //执行定时器，传入当前的时间(毫秒)
    virtual void handle() = 0;
};


#endif
