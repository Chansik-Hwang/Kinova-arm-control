//
// Created by tina on 23. 11. 9.
//

#ifndef EE3100704_PROJECTS_SETOBSTACLE_HPP
#define EE3100704_PROJECTS_SETOBSTACLE_HPP

#include "raisim/World.hpp"


class setObstacle {

public:
    void setSphere(raisim::World* world, float radius, float mass, float x, float y, float z);
};


#endif //EE3100704_PROJECTS_SETOBSTACLE_HPP
