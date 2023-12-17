//
// Created by tina on 23. 11. 7.
//

#ifndef EE3100704_PROJECTS_JOINTCONTROLL_HPP
#define EE3100704_PROJECTS_JOINTCONTROLL_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include "Others/setTime.hpp"
#include "Planning/cubicTrajectoryGenerator.hpp"

class robotController {
public:

    robotController() : JointSpaceInput(6){

    }
    void setInitialState(raisim::ArticulatedSystem* robot, Eigen::VectorXd initialPosition);
    void setPDgain(Eigen::VectorXd Pgain, Eigen::VectorXd Dgain);
    void setFixedBasePosition(raisim::World* world, raisim::ArticulatedSystem* robot, float timeDuration);
    void setFloatingBasePosition(raisim::World* world, raisim::ArticulatedSystem* robot, float timeDuration);
    void setStand(raisim::World* world, raisim::ArticulatedSystem* robot);
    void setSit(raisim::World* world, raisim::ArticulatedSystem* robot);


    Eigen::VectorXd JointSpaceInput;

private:
    void setBasePose();

private:
    float d2r = 3.141592/180;
    int baseQuaternion = 7;

    Eigen::VectorXd basePose;

    Eigen::VectorXd mPgain;
    Eigen::VectorXd mDgain;
};


#endif //EE3100704_PROJECTS_JOINTCONTROLL_HPP
