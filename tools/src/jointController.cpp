//
// Created by tina on 23. 11. 7.
//

#include "jointController.hpp"

void jointController::setInitialState(raisim::ArticulatedSystem *robot, Eigen::VectorXd initialPosition)
{
    robot->setGeneralizedCoordinate(initialPosition);
    sleep(2);
}

void jointController::setPDgain(Eigen::VectorXd Pgain, Eigen::VectorXd Dgain)
{
    mPgain = Pgain;
    mDgain = Dgain;
}

void jointController::setPosition(raisim::ArticulatedSystem *robot, float timeDuration)
{
    cubicTrajectoryGenerator trajectoryGenerator[robot->getDOF()];
    setTime setTime;

    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());

    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim());
    robot->setGeneralizedCoordinate(currentPosition);

    /// set time
    setTime.setTimeInitiallize();
    setTime.timedT = 0.02;

    /// set joint goal position
    for (int i = 0; i < robot->getDOF(); i++)
    {
        std::cout << "input joint " << i+1 << " value (degree) : ";
        std::cin >> goalPosition[i];
    }
    goalPosition = goalPosition*d2r;
    std::cout << goalPosition << std::endl;

    /// create trajectory
    for (int i = 0; i < robot->getDOF(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration)  ;
    }

    while (1)
    {
        setTime.setLocaltime(); //get in while loop.
        for (int jointNum = 0; jointNum < robot->getDOF() ; jointNum++)
        {
            jointPositionTarget[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
            jointVelocityTarget[jointNum] = trajectoryGenerator[jointNum].getVelocityTrajectory(setTime.localtime);
        }

        /// robot set position
        robot->setGeneralizedCoordinate(jointPositionTarget);
        robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
        robot->setPdGains(mPgain, mDgain);
        robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
        usleep(10000);
        if (setTime.localtime == timeDuration)
            break;
    }

}