//
// Created by tina on 23. 11. 6.
//

#include "Others/setTime.hpp"

void setTime::setTimeInitiallize()
{
    localtime = 0;
    timeIteraition = 0;
}

void setTime::setLocaltime()
{
    localtime = timeIteraition * timedT;
    timeIteraition++;
}