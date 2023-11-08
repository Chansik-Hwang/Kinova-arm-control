//
// Created by tina on 23. 11. 8.
//

#ifndef EE3100704_PROJECTS_BASECONTROLLER_HPP
#define EE3100704_PROJECTS_BASECONTROLLER_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include "setTime.hpp"

class baseController {
public:
    Eigen::VectorXd setBasePosition();

private:
    int baseQuaternion = 7;
};


#endif //EE3100704_PROJECTS_BASECONTROLLER_HPP
