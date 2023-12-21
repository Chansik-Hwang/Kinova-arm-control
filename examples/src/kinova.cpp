//
// Created by jy on 23. 11. 1.
//

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "Planning/cubicTrajectoryGenerator.hpp"
#include "Control/robotController.hpp"
#include "Others/setObstacle.hpp"
#include "Control/Control.hpp"

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

    auto endeffectorIndex = kinova->getFrameIdxByLinkName("kinova_end_effector");


    raisim::Mat<3,3> rotationcheck;
    raisim::Vec<3> position;

    raisim::Vec<3> vel;
    raisim::Vec<3> angvel;
    Eigen::VectorXd computedvel(6);
    raisim::VecDyn jointvelocity = raisim::VecDyn(6);
    Eigen::VectorXd jointvel(6);

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.setMap("simple");
    server.launchServer();

    server.focusOn(kinova);
    kinova->setName("kinova");
    cup->setName("cup");
    newtable->setName("newtable");
    std::cout << newtable->getDOF();

//    sleep(2);

    control kinovaControl;
    Eigen::VectorXd FKresult(6);
    Eigen::VectorXd Goalposition(6);          /// degree
//    Goalposition << 0.478747,0.64008, 1.19644, 32.9011,-62.6701,-176.539; ///degree 300 90 120 30 140 0
    Goalposition << -0.708552,-0.0289104, 1.25142, 79.2681,-6.42481,-97.8791; ///degree 300 90 120 30 140 0
    Eigen::VectorXd test(6);
    test << 170, 100, 110, 20, 110, 80;
    test = PI/180 * test;
    /// set controller
    robotController controller;

    Eigen::VectorXd jointPgain(kinova->getDOF()), jointDgain(kinova->getDOF());
    Eigen::VectorXd initialJointPosition(kinova->getGeneralizedCoordinateDim());
    Eigen::VectorXd CupJointPosition(cup->getGeneralizedCoordinateDim());
    Eigen::VectorXd cupVelocity(cup->getDOF());

    cupVelocity.setZero();

    initialJointPosition << 150,80,90,0,90,60;
    initialJointPosition = PI/180 * initialJointPosition;

    CupJointPosition << 0.3,0.7,1.1, 1.57, 0, 0,1;  /// z = 0.454 cup is on the table

    float timeDuration = 1.3; /// 정하는 기준?
    jointPgain << 80.0, 80.0, 80.0, 40.0, 40.0, 40.0;
    jointDgain << 2.0, 2.0, 2.0, 0.5, 0.5, 0.5;

    cup->setGeneralizedCoordinate(CupJointPosition);
    cup->setGeneralizedForce(Eigen::VectorXd::Zero(cup->getDOF()));
    cup->setPdGains(jointPgain, jointDgain);
    cup->setPdTarget(CupJointPosition, cupVelocity);

    controller.setInitialState(kinova, initialJointPosition); /// joint 6개 초기각도 0세팅
    controller.setPDgain(jointPgain,jointDgain); ///각 joint 별로 P,D게인 설정

    /// make trajectory and run
    char run;

//    while(1) {
//        std::cout << "\nStrike the Obstacle(y/n) : ";
//        std::cin >> run;
//        if (run == 'y') {

//    controller.setFixedBasePosition(&world, kinova, timeDuration);
//    kinova->getFrameOrientation(endeffectorIndex, rotationcheck);
//    kinova->getFramePosition(endeffectorIndex, position);
//    kinova->getFrameVelocity(endeffectorIndex, vel);
//    kinova->getFrameAngularVelocity(endeffectorIndex, angvel);

//    FKresult << kinovaControl.ComputeFK(test);

    std::cout << "x y z r p y(degree, by FK) : ";
    FKresult[3] = FKresult[3] * 180 / PI;
    FKresult[4] = FKresult[4] * 180 / PI;
    FKresult[5] = FKresult[5] * 180 / PI;
    for (int i = 0; i < kinova->getDOF(); i++) {
        std::cout << FKresult[i] << "  ";
    }
//    std::cout << std::endl;
//    std::cout << "x y z(by Raisim Function) : " << position[0] << " " << position[1] << " " << position[2] << std::endl;
//    std::cout << "Orientation(by Raisim Function)" << std::endl;
//    std::cout << rotationcheck << std::endl;
//                        std::cout << "1. Task space velocity" << std::endl;
//                        std::cout << vel;
//                        std::cout << angvel << std::endl;
//
//                        jointvelocity = kinova->getGeneralizedVelocity(); ///joint velocity
//                        for(int i=0; i<6; i++){
//                            jointvel[i] = jointvelocity[i]; ///change to Eigen vector
//                        }
//                        std::cout << "2. joint velocity" << std::endl;
//                        std::cout << jointvel << std::endl;
    sleep(2);
    Goalposition = kinovaControl.ComputeIK(initialJointPosition,Goalposition,kinova);
    kinovaControl.JointPDControl(Goalposition, &world, kinova, timeDuration);

//                        std::cout << "3. J*jointvelocity computed" << std::endl;
//                        computedvel << kinovaControl.testJ*jointvel;
//                        std::cout << computedvel << std::endl;

            //            setObstacle.setSphere(&world, 0.04, 1, joint2position[0], joint2position[1], joint2position[2]); for FK position check
//        }
//        else {
//            std::cout << "Entire Process End" << std::endl;
//            break;
//        }
//    }

    for (int i=0; i<2000000; i++)         ///raisim 시간흘러가며 유지]
    {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();
    }

    server.killServer();

}

