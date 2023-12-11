//
// Created by tina on 23. 11. 9.
//

#ifndef EE3100704_PROJECTS_SETOBSTACLE_HPP
#define EE3100704_PROJECTS_SETOBSTACLE_HPP

#include "raisim/World.hpp"


class setObstacle {

public:
    void setSphere(raisim::World* world, float radius, float mass, float x, float y, float z);
    void setBox(raisim::World* world, double xLength, double yLength, double zLength, double x, double y, double z);
//    void setTable1(raisim::World* world);
//    void setTable2(raisim::World* world);
};


#endif //EE3100704_PROJECTS_SETOBSTACLE_HPP
