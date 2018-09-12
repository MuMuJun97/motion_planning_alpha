/*-------------------------------------------------------
 * 文件名：nad_timer.cpp
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：定时器的基类
-------------------------------------------------------*/


//头文件
#include "nad_timer.h"


//构造定时器
nad_timer::nad_timer(int64 interval_ms)
{
    this->interval_ms = interval_ms;
    this->last_ms = 0;
}

//析构函数
nad_timer::~nad_timer()
{
}