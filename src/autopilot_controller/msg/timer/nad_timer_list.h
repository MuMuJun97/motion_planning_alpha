/*-------------------------------------------------------
 * 文件名：nad_timer_list.h
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：定时器列表
-------------------------------------------------------*/
#ifndef _nad_timer_list_H
#define _nad_timer_list_H


#include "nad_timer.h"


//定时器管理器的基类
class nad_timer_list
{
//protected:
public:
    //定时器列表
    vector<nad_timer *> timer_list;

public:
    //构造析构函数
    nad_timer_list();
    virtual ~nad_timer_list();

    //新增定时器
    int add_timer(nad_timer *timer);

    //删除定时器
    int delete_timer(nad_timer *timer);

    //执行定时器，传入当前的时间(毫秒)
    void handle_timer();
};

//全局定时器数组
extern nad_timer_list g_ltimer;


#endif
