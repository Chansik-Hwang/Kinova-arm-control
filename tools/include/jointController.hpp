//
// Created by tina on 23. 11. 7.
//

#ifndef EE3100704_PROJECTS_JOINTCONTROLL_HPP
#define EE3100704_PROJECTS_JOINTCONTROLL_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include "setTime.hpp"
#include "cubicTrajectoryGenerator.hpp"

class jointController {
public:
    void setInitialState(raisim::ArticulatedSystem* robot, Eigen::VectorXd initialPosition);
    void setPDgain(Eigen::VectorXd Pgain, Eigen::VectorXd Dgain);
    void setPosition(raisim::ArticulatedSystem* robot, float timeDuration);

private:
    float d2r = 3.141592/180;
    Eigen::VectorXd mPgain;
    Eigen::VectorXd mDgain;
};


#endif //EE3100704_PROJECTS_JOINTCONTROLL_HPP
