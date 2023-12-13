//
// Created by chansik on 23. 12. 11.
//

#ifndef EE3100704_PROJECTS_CONTROL_HPP
#define EE3100704_PROJECTS_CONTROL_HPP
#define PI 3.141592

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <vector>
class control{

private:
    Eigen::Matrix3d R01,R12,R23,R34,R45,R56,R67;
    Eigen::Matrix4d T01,T12,T23,T34,T45,T56,T67;

    Eigen::Matrix3d rotate(double r, double p, double y);
    void check_pitch(Eigen::Matrix3d R12,Eigen::Matrix3d R23,Eigen::Matrix3d R45);
    void check_yaw(Eigen::Matrix3d R01,Eigen::Matrix3d R34,Eigen::Matrix3d R56);
    void changeR2T(Eigen::Matrix3d R01,Eigen::Matrix3d R12,Eigen::Matrix3d R23,
                   Eigen::Matrix3d R34,Eigen::Matrix3d R45,Eigen::Matrix3d R56, Eigen::Matrix3d R67);

public:

    std::vector<double> ComputeFK(std::vector<double> &inputjoints);
    std::vector<double> FKvalue;
//    void changeRT(Eigen::MatrixXd R);


};










#endif //EE3100704_PROJECTS_CONTROL_HPP
