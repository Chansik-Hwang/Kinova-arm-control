//
// Created by jy on 23. 11. 1.
//

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "cubicTrajectoryGenerator.hpp"
#include "robotController.hpp"

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

    /// set controller
    robotController controller;

    Eigen::VectorXd jointPgain(kinova->getDOF()), jointDgain(kinova->getDOF());
    Eigen::VectorXd initialJointPosition(kinova->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    float timeDuration = 3.0;

    jointPgain << 40.0, 40.0, 40.0, 15.0, 15.0, 15.0;
    jointDgain << 2.0, 2.0, 2.0, 0.5, 0.5, 0.5;

    controller.setInitialState(kinova, initialJointPosition);
    controller.setPDgain(jointPgain,jointDgain);
    controller.setFixedBasePosition(&world, kinova,timeDuration);

    /// make trajectory and run
    char run;
    while (1)
    {
        std::cout << "\nDo you want to keep going? [y/n]  ";
        std::cin >> run;
        if (run == 'y')
        {
            controller.setFixedBasePosition(&world, kinova,timeDuration);
        }
        else
        {
            std::cout << "Bye. Please quit. " << std::endl;
            break;
        }
    }

    for (int i=0; i<2000000; i++)
    {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();
    }

    server.killServer();

}

