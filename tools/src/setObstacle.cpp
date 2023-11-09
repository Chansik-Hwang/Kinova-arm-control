//
// Created by tina on 23. 11. 9.
//

#include "setObstacle.hpp"

void setObstacle::setSphere(raisim::World* world, float radius, float mass, float x, float y, float z)
{
    auto ball = world->addSphere(radius,mass,"rubber");
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d angularVelocity;
    linearVelocity << 0,0,-1;
    angularVelocity.setZero();
    ball->setPosition(x,y,z);
    ball->setVelocity(linearVelocity, angularVelocity);
}