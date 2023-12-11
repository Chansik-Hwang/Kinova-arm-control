//
// Created by tina on 23. 11. 9.
//

#include "Others/setObstacle.hpp"

void setObstacle::setSphere(raisim::World* world, float radius, float mass, float x, float y, float z)
{
    auto ball = world->addSphere(radius,mass,"rubber");
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d angularVelocity;
    linearVelocity << 0,0,-1;
    angularVelocity.setZero();
    ball->setPosition(x,y,z);
    ball->setVelocity(linearVelocity, angularVelocity);
    ball->setAppearance("red");
}

void setObstacle::setBox(raisim::World* world, double xLength, double yLength, double zLength, double x, double y, double z)
{
    auto box = world->addBox(xLength,yLength,zLength,1.0,"steel", raisim::COLLISION(3),-1);
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d angularVelocity;
    linearVelocity << 0,0,0;
    angularVelocity.setZero();
    box->setPosition(x,y,z);
    box->setVelocity(linearVelocity, angularVelocity);
}

//void setTable1(raisim::World* world)
//{
//    setBox(&world,0.1,0.1,0.4,0.7,0.5,0.2);
//    setBox(&world,0.1,0.1,0.4,-0.7,0.5,0.2);
//    setBox(&world,0.1,0.1,0.4,0.7,1,0.2);
//    setBox(&world,0.1,0.1,0.4,-0.7,1,0.2);
//    setBox(&world,1.5,0.59,0.05,0,0.75,0.425);
//}

//void setTable2(raisim::World* world)
//{
//
//}