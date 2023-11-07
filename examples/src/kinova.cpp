//
// Created by jy on 23. 11. 1.
//

#include <unistd.h>
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "cubicTrajectoryGenerator.hpp"
#include "setTime.hpp"

int main(int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    world.addGround();
    auto kinova = world.addArticulatedSystem("/home/tina/EE3100704/examples/rsc/kinova/urdf/kinova.urdf");

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.setMap("simple");
    server.launchServer();

    server.focusOn(kinova);
    kinova->setName("kinova");

    /// set time
    setTime setTime;
    setTime.setTimeInitiallize();
    setTime.timedT = 0.02;

    /// kinova joint PD controller
    Eigen::VectorXd jointPositionTarget(kinova->getGeneralizedCoordinateDim()), jointVelocityTarget(kinova->getDOF());
    Eigen::VectorXd jointPgain(kinova->getDOF()), jointDgain(kinova->getDOF());
    jointPgain << 40.0, 40.0, 40.0, 15.0, 15.0, 15.0;
    jointDgain << 2.0, 2.0, 2.0, 0.5, 0.5, 0.5;

    ///set joint initial state
    cubicTrajectoryGenerator trajectoryGenerator[kinova->getDOF()];
    double timeDuration = 3.0;
    Eigen::VectorXd jointGoalPosition(kinova->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointPosition(kinova->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();

    kinova->setPdGains(jointPgain, jointDgain);
    kinova->setPdTarget(jointPositionTarget, jointVelocityTarget);
    kinova->setGeneralizedCoordinate(initialJointPosition);
    sleep(5);

    /// set joint goal position

    jointGoalPosition << 0.0, 3.141592, -1.57, 0.0, 2.0, 0.0;

    std::cout <<  kinova->getGeneralizedCoordinate() << std::endl;

    /// create trajectory
    for (int i = 0; i < kinova->getDOF(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(initialJointPosition[i],jointGoalPosition[i],setTime.localtime,timeDuration)  ;
    }

    while (1)
    {
        setTime.setLocaltime(); //get in while loop.
        for (int jointNum = 0; jointNum < kinova->getDOF() ; jointNum++)
        {
            jointPositionTarget[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
            jointVelocityTarget[jointNum] = trajectoryGenerator[jointNum].getVelocityTrajectory(setTime.localtime);
        }

        /// kinova set position
        kinova->setGeneralizedCoordinate(jointPositionTarget);
        usleep(10000);
        std::cout << kinova->getGeneralizedCoordinate() << std::endl;
        if (setTime.localtime == timeDuration)
            break;
    }

    kinova->setPdGains(jointPgain, jointDgain);
    kinova->setPdTarget(jointPositionTarget, jointVelocityTarget);

    std::cout << kinova->getGeneralizedCoordinate() << std::endl;


    for (int i=0; i<2000000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();
    }

    server.killServer();
}

double getAccelerationTrajectory(double currentTime)
{
    return 0;
}
