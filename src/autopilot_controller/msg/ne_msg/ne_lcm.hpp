#ifndef __NE_lcm_hpp__
#define __NE_lcm_hpp__


#include <lcm/lcm-cpp.hpp>
#include "ne_msg_t.hpp"

#include <iostream>

class NE_LCM: public lcm::LCM
{
public:

    NE_LCM(std::string lcm_url) : lcm::LCM (lcm_url)
    {
    }

    //网元间消息用publish_nemsg发送
    //网元内消息用原始接口publish发送
    template <class T>
    int publish_nemsg(T &msg)
    {
        msg.encode_body();
        //return publish("NE_MSG", &msg);//这是正式版的代码

        if(msg.header.local_ne_name.compare("csu") == 0)
        {
            return publish("NEMSG_CSU", &msg);//这是正式版的代码
        }
        else if(msg.header.local_ne_name.compare("rsu_1") == 0)
        {
            return publish("NEMSG_RSU", &msg);//这是正式版的代码
        }
        else
        {
            return publish("NEMSG_OBU", &msg);//这是正式版的代码
        }

        //目前调试阶段，直接写入目标channel
        //return publish(msg.header.peer_channel, &msg);
    };

    int async_handle(struct timeval tv)
    {
        int ret = 0;
        int lcm_fd = getFileno();
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(lcm_fd, &readfds);
        int status = 0;
        status = select (lcm_fd+1, &readfds, NULL, NULL, &tv);
        if (status < 0)
        {
            std::cout<< "ERROR! select failed\n"<< std::endl;
            return ret;
        }
        if (FD_ISSET (lcm_fd,&readfds))
        {
             ret = handle();
        }
        return ret;
    }
};

#endif
