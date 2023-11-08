//
// Created by jy on 23. 11. 1.
//

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "cubicTrajectoryGenerator.hpp"
#include "robotController.hpp"
#include "baseController.hpp"

int main(int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    auto ground = world.addGround();
    ground->setAppearance("steel");
    auto canine = world.addArticulatedSystem("/home/tina/EE3100704/examples/rsc/canine/urdf/canineV4_2.urdf");

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(canine);
    server.launchServer();
    canine->setName("canine");

    /// set joint Initialization
    Eigen::VectorXd initialJointPosition(canine->getGeneralizedCoordinateDim()), jointVelocityTarget(canine->getDOF());
    initialJointPosition << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    jointVelocityTarget.setZero();

    Eigen::VectorXd jointPgain(canine->getDOF()), jointDgain(canine->getDOF());
    jointPgain.tail(12).setConstant(100.0);
    jointDgain.tail(12).setConstant(1.0);
    sleep(2);

    canine->setGeneralizedCoordinate(initialJointPosition);
    canine->setGeneralizedForce(Eigen::VectorXd::Zero(canine->getDOF()));
    canine->setPdGains(jointPgain, jointDgain);
    canine->setPdTarget(initialJointPosition, jointVelocityTarget);

    float timeDuration = 3.0;

    /// set controller
    robotController controller;

    controller.setPDgain(jointPgain,jointDgain);
    controller.setFloatingBasePosition(&world, canine, timeDuration);
/*
    /// make trajectory and run
    char run;
    while (1)
    {
        std::cout << "\nDo you want to keep going? [y/n]  ";
        std::cin >> run;
        if (run == 'y')
        {
            controller.setPosition(&world, canine,timeDuration);
        }
        else
        {
            std::cout << "Bye. Please quit. " << std::endl;
            break;
        }
    }
*/
    for (int i=0; i<2000000; i++)
    {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();
    }

    server.killServer();

}

