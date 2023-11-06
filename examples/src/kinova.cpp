//
// Created by jy on 23. 11. 1.
//

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

    /// set time
    setTime setTime;
    setTime.setTimeInitiallize();
    setTime.timedT = 0.02;

    /// create trajectory
    cubicTrajectoryGenerator trajectoryGenerator[kinova->getDOF()];
    double timeDuration = 1.0;
    Eigen::VectorXd jointGoalPosition = Eigen::VectorXd (kinova->getDOF());
    Eigen::VectorXd jointCurrentPosition = Eigen::VectorXd (kinova->getDOF());

    jointGoalPosition << 0.0, 0.0, 0.0, 0.0, 2.0, 0.0;
    jointCurrentPosition << 0.0, 2.76, -1.57, 0.0, 2.0, 0.0;
    for (int i = 0; i < kinova->getDOF(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(jointCurrentPosition[i],jointGoalPosition[i],setTime.localtime,timeDuration)  ;
    }

    /// kinova joint PD controller
    Eigen::VectorXd jointNominalConfig(kinova->getGeneralizedCoordinateDim()), jointVelocityTarget(kinova->getDOF());
    jointNominalConfig << 0.0, 2.76, -1.57, 0.0, 2.0, 0.0;
    jointVelocityTarget.setZero();

    Eigen::VectorXd jointPgain(kinova->getDOF()), jointDgain(kinova->getDOF());
    jointPgain << 40.0, 40.0, 40.0, 15.0, 15.0, 15.0;
    jointDgain << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;

    kinova->setPdGains(jointPgain, jointDgain);
    kinova->setPdTarget(jointNominalConfig, jointVelocityTarget);

    while (1)
    {
        setTime.setLocaltime(); //get in while loop.
        for (int jointNum = 0; jointNum < kinova->getDOF() ; jointNum++)
        {
            jointNominalConfig[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
            jointVelocityTarget[jointNum] = trajectoryGenerator[jointNum].getVelocityTrajectory(setTime.localtime);
        }

        std::cout << jointNominalConfig << std::endl;

        /// kinova set torque
        kinova->setGeneralizedCoordinate(jointNominalConfig);
//        kinova->setGeneralizedForce(Eigen::VectorXd::Zero(kinova->getDOF()));
        if (setTime.localtime == timeDuration)
            break;
    }


    kinova->setName("kinova");

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.setMap("simple");
    server.launchServer();
    server.focusOn(kinova);

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
