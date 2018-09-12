#ifndef SRC_AX7CONTROLLER_HPP_
#define SRC_AX7CONTROLLER_HPP_

#include "Controller.hpp"

class AX7controller : public Controller {
public:
    AX7controller(const std::string &inifilename, const std::string &controller_name,
                  const int controller_code);

    virtual ~AX7controller();
};

#endif /* SRC_AX7CONTROLLER_HPP_ */
