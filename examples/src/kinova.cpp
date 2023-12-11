//
// Created by jy on 23. 11. 1.
//

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "Planning/cubicTrajectoryGenerator.hpp"
#include "Control/robotController.hpp"
#include "Others/setObstacle.hpp"

int main(int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);
//    world.setGravity({ 0.0, 0.0, -9.81 });

    /// create objects
    world.addGround();
    auto kinova = world.addArticulatedSystem("/home/chansik/EE3100704/examples/rsc/kinova/urdf/kinova.urdf"); //set your path
    auto cup = world.addArticulatedSystem("/home/chansik/EE3100704/examples/rsc/kinova/urdf/cup.urdf");
    auto newtable = world.addArticulatedSystem("/home/chansik/EE3100704/examples/rsc/kinova/urdf/newtable.urdf");
    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.setMap("simple");
    server.launchServer();

    server.focusOn(kinova);
    kinova->setName("kinova");
    cup->setName("cup");
    newtable->setName("newtable");
    std::cout << newtable->getDOF();

    sleep(2);

    /// set obstacle
    setObstacle setObstacle;

    setObstacle.setBox(&world,0.3,0.3,0.4,0.7,-0.7,0.2);
    setObstacle.setBox(&world,0.3,0.3,0.4,-0.7,-0.7,0.2);
    setObstacle.setBox(&world,1.4,0.3,0.05,0,-0.7,0.425);

//    setObstacle.setSphere(&world, 0.05, 1, 0.2, 0.8, 0.5);

    /// set controller
    robotController controller;

    Eigen::VectorXd jointPgain(kinova->getDOF()), jointDgain(kinova->getDOF());
    Eigen::VectorXd initialJointPosition(kinova->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointPosition2(cup->getGeneralizedCoordinateDim());
//    Eigen::VectorXd Table_initialJointPosition(newtable->getGeneralizedCoordinateDim());
    Eigen::VectorXd cupVelocity(cup->getDOF());

    cupVelocity.setZero();

    initialJointPosition << 1.22173,1.22173,1.22173,1.22173,1.22173,0;
    initialJointPosition2 << 0, 0.2, 0.0, 1.57, 0, 0,1;
//    Table_initialJointPosition.setZero();

    float timeDuration = 3.0; /// 정하는 기준?
    jointPgain << 80.0, 80.0, 80.0, 40.0, 40.0, 40.0;
    jointDgain << 2.0, 2.0, 2.0, 0.5, 0.5, 0.5;


    cup->setGeneralizedCoordinate(initialJointPosition2);
    cup->setGeneralizedForce(Eigen::VectorXd::Zero(cup->getDOF()));
    cup->setPdGains(jointPgain, jointDgain);
    cup->setPdTarget(initialJointPosition2, cupVelocity);


    controller.setInitialState(kinova, initialJointPosition); /// joint 6개 초기각도 0세팅
    controller.setPDgain(jointPgain,jointDgain); ///각 joint 별로 P,D게인 설정
    controller.setFixedBasePosition(&world, kinova,timeDuration); ///




    /// make trajectory and run
    char run;
    while (1)
    {
        std::cout << "\nDo you want to keep going? [y/n]  ";
        std::cin >> run;
        if (run == 'y')
        {
            controller.setFixedBasePosition(&world, kinova, timeDuration);
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

