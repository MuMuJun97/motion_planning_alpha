/*-------------------------------------------------------
 * 文件名：nad_session_list.pp
 * 创建者：张毅00151602
 * 时  间：2016-03-02
 * 描  述：SESSION(控制块)列表
-------------------------------------------------------*/


//头文件
#include "nad_session_list.h"
#include "../ne_msg/ne_msg_base_t.hpp"

//定时器
class nad_session_timer : public nad_timer
{
private:
    //需要定时处理的session列表
    nad_session_list *session_list;

public:
    //构造析构函数
    nad_session_timer(int64 interval_ms, nad_session_list *session_list);
    virtual ~nad_session_timer();

    //执行定时器，传入当前的时间(毫秒)
    void handle();
};

//构造函数
nad_session_timer::nad_session_timer(int64 interval_ms, nad_session_list *session_list) : nad_timer(interval_ms)
{
    this->session_list = session_list;
}

//析构函数
nad_session_timer::~nad_session_timer()
{
}

//执行定时器，传入当前的时间(毫秒)
void nad_session_timer::handle()
{
    session_list->handle_timer();
}


//构造函数，参数(最大session数&0=无限, 定时器间隔&0=不起定时器)
nad_session_list::nad_session_list(int max_size, int64 interval_ms)
{
    this->max_size = (max_size <= 0) ? MAX_I32 : max_size;
    if (interval_ms > 0)
    {
        timer = new nad_session_timer(interval_ms, this);
        cout << "nad_session_list::nad_session_list(int max_size, int64 interval_ms) :" << g_ltimer.timer_list.size() << endl;
        g_ltimer.add_timer(timer);
        cout << "nad_session_list::nad_session_list(int max_size, int64 interval_ms) :" << g_ltimer.timer_list.size() << endl;
    }
    else
    {
        timer = NULL;
    }
}

//析构函数
nad_session_list::~nad_session_list()
{
    clear_session();
    if (timer != NULL)
    {
        g_ltimer.delete_timer(timer);
    }
}

//获得session数量，禁止重载
int nad_session_list::size()
{
    return session_map.size();
}

//新增SESSION(控制块)，session->name不能重复!
int nad_session_list::add_session(nad_session *session)
{
    nad_session *result = find_session(session->name);
    if (result == NULL)
    {
        //没找到，则添加
        cout << "nad_session_list::add_session(" << session->name << "): ok" << endl;
        session_map[session->name] = session;
        return RET_OK;
    }
    else
    {
        //找到
        cout << "nad_session_list::add_session(" << session->name << "): exists" << endl;
        return RET_EXIST;
    }
}

//查询SESSION(控制块)，name不能重复!
nad_session *nad_session_list::find_session(string name)
{
    map<string, nad_session*>::iterator iter;
    iter = session_map.find(name);
    if (iter == session_map.end())
    {
        return NULL;
    }
    else
    {
        return iter->second;
    }
}

//删除SESSION(控制块)，成功返回ERT_OK
int nad_session_list::delete_session(string name)
{
    map<string, nad_session*>::iterator iter;
    iter = session_map.find(name);
    if (iter == session_map.end())
    {
        cout << "nad_session_list::delete_session(" << name << "): not exists" << endl;
        return RET_NOT_EXIST;
    }
    else
    {
        cout << "nad_session_list::delete_session(" << name << "): ok" << endl;
        delete iter->second;
        session_map.erase(iter);
        return RET_OK;
    }
}

//删除所有SESSION(控制块)
int nad_session_list::clear_session()
{
    map<string, nad_session*>::iterator iter;
    for(iter = session_map.begin(); iter != session_map.end(); iter++)
    {
        delete iter->second;
    }
    session_map.clear();
}

//处理消息，如果收到的不是ne_msg_t必须重载，否则内部强转会coredump
int nad_session_list::handle_msg(const void *nemsg)
{
    const nad_lcm::ne_msg_base_t *msg = (const nad_lcm::ne_msg_base_t *)nemsg;
    nad_session *session = find_session(msg->header.session_name);
    if (session != NULL)
    {
        session->handle_msg(nemsg);
    }
}

//处理定时器
void nad_session_list::handle_timer()
{
    map<string, nad_session*>::iterator  iter;
    for(iter = session_map.begin(); iter != session_map.end(); iter++)
    {
        nad_session *session = iter->second;
        session->handle_timer();
    }
}

//注册消息到lcm
void nad_session_list::reg_msg_to_lcm(NE_LCM *lcm)
{
}
