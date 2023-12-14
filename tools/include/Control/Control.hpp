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
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
class control{

private:
    Eigen::Matrix3d R01,R12,R23,R34,R45,R56,R67;
    Eigen::Matrix4d T01,T12,T23,T34,T45,T56,T67;
    double d2r = PI/180;

    Eigen::MatrixXd Jacobian_v;
    Eigen::MatrixXd Jacobian_w;
    Eigen::MatrixXd J;

    Eigen::Matrix3d rotate(double r, double p, double y);
    void check_pitch(Eigen::Matrix3d R12,Eigen::Matrix3d R23,Eigen::Matrix3d R45);
    void check_yaw(Eigen::Matrix3d R01,Eigen::Matrix3d R34,Eigen::Matrix3d R56);
    void changeR2T(Eigen::Matrix3d R01,Eigen::Matrix3d R12,Eigen::Matrix3d R23,
                   Eigen::Matrix3d R34,Eigen::Matrix3d R45,Eigen::Matrix3d R56, Eigen::Matrix3d R67);

    void getJacobian(std::vector<double> &th);

public:
    control() : Jacobian_v(3, 6),Jacobian_w(3, 6),J(6, 6){

    }


    std::vector<double> FKvalue;

    std::vector<double> ComputeFK(std::vector<double> &inputjoints);
    void ComputeIK(std::vector<double> &th, raisim::ArticulatedSystem *robot);




};










#endif //EE3100704_PROJECTS_CONTROL_HPP
