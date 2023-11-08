//
// Created by tina on 23. 11. 8.
//

#include "baseController.hpp"

Eigen::VectorXd baseController::setBasePosition()
{

    Eigen::VectorXd basePosition(baseQuaternion);

    /// set base goal position
    std::cout << "\ninput base x value ";
    std::cin >> basePosition[0];
    std::cout << "\ninput base y value ";
    std::cin >> basePosition[1];
    std::cout << "\ninput base z value ";
    std::cin >> basePosition[2];
    std::cout << "\ninput base w value ";
    std::cin >> basePosition[3];

    std::cout << basePosition << std::endl;

    return basePosition;

}