//
// Created by chansik on 23. 12. 11.
//

#ifndef EE3100704_PROJECTS_CONTROL_HPP
#define EE3100704_PROJECTS_CONTROL_HPP
#define PI 3.14159265358979

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <vector>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "Others/setTime.hpp"
#include "Planning/cubicTrajectoryGenerator.hpp"
class control{

private:
    Eigen::Matrix3d R01,R12,R23,R34,R45,R56,R67;
    Eigen::Matrix4d T01,T12,T23,T34,T45,T56,T67;
    double d2r = PI/180.0;
    double r2d = 180.0/PI;

    Eigen::MatrixXd Jacobian_v;
    Eigen::MatrixXd Jacobian_w;
    Eigen::MatrixXd J;
    Eigen::MatrixXd Inv_J;

    Eigen::VectorXd mPgain;
    Eigen::VectorXd mDgain;

    int LoopCount;


    Eigen::Matrix3d rotate(double r, double p, double y);
    void changeR2T(Eigen::Matrix3d R01,Eigen::Matrix3d R12,Eigen::Matrix3d R23,
                   Eigen::Matrix3d R34,Eigen::Matrix3d R45,Eigen::Matrix3d R56, Eigen::Matrix3d R67);
    void getOrientation(Eigen::MatrixXd Tmatrix, Eigen::VectorXd &FKValue);
    void getJacobian(Eigen::VectorXd &th);

public:
    control() : Jacobian_v(3, 6),Jacobian_w(3, 6),J(6, 6), Inv_J(6,6),testJ(6,6),
                mPgain(6),mDgain(6){

    }

    void JointPDControl(Eigen::VectorXd Joints, raisim::World* world, raisim::ArticulatedSystem *robot, float timeDuration);
    Eigen::VectorXd ComputeFK(Eigen::VectorXd &inputjoints);
    Eigen::VectorXd ComputeIK(Eigen::VectorXd &initial_angle, Eigen::VectorXd goal_pose,raisim::ArticulatedSystem *robot);

    Eigen::MatrixXd testJ;




};










#endif //EE3100704_PROJECTS_CONTROL_HPP
