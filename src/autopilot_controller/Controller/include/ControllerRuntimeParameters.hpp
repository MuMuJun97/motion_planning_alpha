#ifndef CONTROLLERRUNTIMEPARAMETERS_HPP_
#define CONTROLLERRUNTIMEPARAMETERS_HPP_

#include <string>
#include <cstdint>

struct ControllerRuntimeParameters {
    std::string map_filename;
    std::string log_dir;
    unsigned int draw_step_points;

    std::string plotter_ip;
    unsigned int plotter_port;

    std::string actuator_ip;
    unsigned int actuator_port;
    std::string actuator_start_msg;
    std::string actuator_stop_msg;

    std::string ins_topic_name;
    std::string way_points_topic_name;

    bool load_local_way_points_map;

    ControllerRuntimeParameters(const std::string &inifilename);
};

#endif /* CONTROLLERRUNTIMEPARAMETERS_HPP_ */
