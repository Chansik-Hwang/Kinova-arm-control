//
// Created by chansik on 23. 12. 11.
//
#include "Control/Control.hpp"



Eigen::VectorXd control::ComputeFK(Eigen::VectorXd &inputjoints){ ///void 말고 joint

    Eigen::MatrixXd result;
    Eigen::VectorXd FKvalue(6);

    for(int i=0; i<6; i++){
        inputjoints[i] = d2r*inputjoints[i];
    }

    R01=rotate(0,0,-inputjoints[0]); ///base to L1+
    R12=rotate(0,-inputjoints[1],0); /// L1 to L2
    R23=rotate(0,inputjoints[2],0); /// L2 to L3
    R34=rotate(0,0,-inputjoints[3]); /// L3 to L4
    R45=rotate(0,-inputjoints[4],0); /// L4 to L5
    R56=rotate(0,0,inputjoints[5]); /// L5 to L6
    R67=rotate(3.14,0,0); /// L6 to End-effector

//    R01=rotate(0,180,0)*rotate(0,0,inputjoints[0]); ///base to L1
//    R12=rotate(0,0,180)*rotate(-90,0,0)*rotate(0,0,inputjoints[1]); /// L1 to L2
//    R23=rotate(0,180,0)*rotate(0,0,inputjoints[2]); /// L2 to L3
//    R34=rotate(0,0,180)*rotate(-90,0,0)*rotate(0,0,inputjoints[3]); /// L3 to L4
//    R45=rotate(0,0,180)*rotate(90,0,0)*rotate(0,0,inputjoints[4]); /// L4 to L5
//    R56=rotate(0,0,180)*rotate(-90,0,0)*rotate(0,0,inputjoints[5]); /// L5 to L6
//    R67=rotate(180,0,0); /// L6 to End-effector


    changeR2T(R01,R12,R23,R34,R45,R56,R67);

    result = T01*T12*T23*T34*T45*T56*T67;

    for(int i=0; i<3; i++){
        FKvalue[i] = (result(i,3));
    }

    check_pitch(R23,R34,R56,FKvalue);
    check_yaw(R12,R45,R67,FKvalue);
    std::cout << result << std::endl;
 std::cout << "=======================" << std::endl;

    return FKvalue;
}

Eigen::Matrix3d control::rotate(double r, double p, double y){

    Eigen::Matrix3d roll;
    Eigen::Matrix3d pitch;
    Eigen::Matrix3d yaw;
    Eigen::Matrix3d I;

    if(r!=0 && p==0 && y==0){
        roll << 1,         0,          0,
                0, cos(r), -sin(r),
                0, sin(r),  cos(r);
        return roll;
    }
    else if(r==0 && p!=0 && y==0){
        pitch << cos(p), 0, sin(p),
                         0, 1,         0,
                -sin(p), 0, cos(p);
        return pitch;
    }
    else if(r==0 && p==0 && y!=0){
        yaw << cos(y), -sin(y), 0,
                sin(y),  cos(y), 0,
                0,          0, 1;
        return yaw;
    }
    else if(r==0 && p==0 && y==0){
        I = Eigen::Matrix3d::Identity();
        return I;
    }

}

void control::changeR2T(Eigen::Matrix3d R01,Eigen::Matrix3d R12,Eigen::Matrix3d R23,
               Eigen::Matrix3d R34,Eigen::Matrix3d R45,Eigen::Matrix3d R56, Eigen::Matrix3d R67){

    Eigen::Vector3d T_01(0,0,0.70675);
    Eigen::Vector3d T_12(0,0.0016,0.11875); ///local 기준으로 돌리고 이동시켜서, T matrix가 안맞다??
    Eigen::Vector3d T_23(0,0,-0.410);
    Eigen::Vector3d T_34(0,-0.0114,0.2073);
    Eigen::Vector3d T_45(0,0,0.10375);
    Eigen::Vector3d T_56(0,0,-0.10375);
    Eigen::Vector3d T_67(0,0,-0.16);

//    Eigen::Vector3d T_01(0,0,0.70675);
//    Eigen::Vector3d T_12(0,0.0016,-0.11875);
//    Eigen::Vector3d T_23(0,-0.410,0);
//    Eigen::Vector3d T_34(0,0.2073,-0.0114);
//    Eigen::Vector3d T_45(0,0,-0.10375);
//    Eigen::Vector3d T_56(0,0.10375,0);
//    Eigen::Vector3d T_67(0,0,-0.16);

    T01 << R01,T_01,
            0,0,0,1;

    T12 << R12,T_12,
            0,0,0,1;

    T23 << R23,T_23,
            0,0,0,1;

    T34 << R34,T_34,
            0,0,0,1;

    T45 << R45,T_45,
            0,0,0,1;

    T56 << R56,T_56,
            0,0,0,1;

    T67 << R67,T_67,
            0,0,0,1;

}

void control::check_pitch(Eigen::Matrix3d R12,Eigen::Matrix3d R23,Eigen::Matrix3d R45,Eigen::VectorXd &FKvalue){

    Eigen::Matrix3d check_pitch = R12*R23*R45;
    double pitch_degree;
    double r2d = 180/PI;

    pitch_degree = r2d*asin(check_pitch(0,2)); ///V
    FKvalue(3) = 3.141592;
    FKvalue[4] = (pitch_degree); ///
    std::cout << std::endl;
    std::cout << "pitch rotate : " << pitch_degree << std::endl;

}

void control::check_yaw(Eigen::Matrix3d R01,Eigen::Matrix3d R34,Eigen::Matrix3d R56,Eigen::VectorXd &FKvalue){

    Eigen::Matrix3d check_yaw = R01*R34*R56;
    double yaw_degree;
    double r2d = 180/PI;

    yaw_degree = r2d*asin(check_yaw(1,0));
    FKvalue[5] = (yaw_degree); ///
    std::cout << "yaw rotate : " << yaw_degree << std::endl;
}


void control::ComputeIK(Eigen::VectorXd &initial_angle, Eigen::VectorXd goal_pose,raisim::ArticulatedSystem *robot){

    getJacobian(initial_angle);  ///update jacobian for every tik, initial angle : radian
    double epsilon = 0.000001;
//    compute_pseudoInverse(J);






}

void control::getJacobian(Eigen::VectorXd &th){


    Eigen::MatrixXd Jv_col1(3,1);
    Eigen::MatrixXd Jv_col2(3,1);
    Eigen::MatrixXd Jv_col3(3,1);
    Eigen::MatrixXd Jv_col4(3,1);
    Eigen::MatrixXd Jv_col5(3,1);
    Eigen::MatrixXd Jv_col6(3,1);

    Eigen::MatrixXd Jw_col1(3,1);
    Eigen::MatrixXd Jw_col2(3,1);
    Eigen::MatrixXd Jw_col3(3,1);
    Eigen::MatrixXd Jw_col4(3,1);
    Eigen::MatrixXd Jw_col5(3,1);
    Eigen::MatrixXd Jw_col6(3,1);

    Jv_col1 << 0.26375*cos(th[4])*(cos(th[1])*sin(th[0])*sin(th[2]) - 1.0*cos(th[2])*sin(th[0])*sin(th[1])) - 0.41*sin(th[0])*sin(th[1]) - 0.26375*sin(th[4])*(cos(th[0])*sin(th[3]) + cos(th[3])*(sin(th[0])*sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0]))) - 0.0098*cos(th[0]) - 0.31105*cos(th[1])*sin(th[0])*sin(th[2]) + 0.31105*cos(th[2])*sin(th[0])*sin(th[1]),
            0.0098*sin(th[0]) - 0.41*cos(th[0])*sin(th[1]) + 0.26375*sin(th[4])*(sin(th[0])*sin(th[3]) - 1.0*cos(th[3])*(cos(th[0])*sin(th[1])*sin(th[2]) + cos(th[0])*cos(th[1])*cos(th[2]))) + 0.26375*cos(th[4])*(cos(th[0])*cos(th[1])*sin(th[2]) - 1.0*cos(th[0])*cos(th[2])*sin(th[1])) - 0.31105*cos(th[0])*cos(th[1])*sin(th[2]) + 0.31105*cos(th[0])*cos(th[2])*sin(th[1]),
            0.0;

    Jv_col2 << 0.41*cos(th[0])*cos(th[1]) + 0.26375*cos(th[4])*(cos(th[0])*sin(th[1])*sin(th[2]) + cos(th[0])*cos(th[1])*cos(th[2])) - 0.31105*cos(th[0])*sin(th[1])*sin(th[2]) + 0.26375*cos(th[3])*sin(th[4])*(cos(th[0])*cos(th[1])*sin(th[2]) - 1.0*cos(th[0])*cos(th[2])*sin(th[1])) - 0.31105*cos(th[0])*cos(th[1])*cos(th[2]),
            0.31105*sin(th[0])*sin(th[1])*sin(th[2]) - 0.26375*cos(th[4])*(sin(th[0])*sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - 0.41*cos(th[1])*sin(th[0]) - 0.26375*cos(th[3])*sin(th[4])*(cos(th[1])*sin(th[0])*sin(th[2]) - 1.0*cos(th[2])*sin(th[0])*sin(th[1])) + 0.31105*cos(th[1])*cos(th[2])*sin(th[0]),
            0.41*sin(th[1]) - 0.26375*cos(th[4])*(cos(th[1])*sin(th[2]) - 1.0*cos(th[2])*sin(th[1])) + 0.31105*cos(th[1])*sin(th[2]) - 0.31105*cos(th[2])*sin(th[1]) + 0.26375*cos(th[3])*sin(th[4])*(cos(th[1])*cos(th[2]) + sin(th[1])*sin(th[2]));



    Jv_col3 << 0.31105*cos(th[0])*sin(th[1])*sin(th[2]) - 0.26375*cos(th[4])*(cos(th[0])*sin(th[1])*sin(th[2]) + cos(th[0])*cos(th[1])*cos(th[2])) - 0.26375*cos(th[3])*sin(th[4])*(cos(th[0])*cos(th[1])*sin(th[2]) - 1.0*cos(th[0])*cos(th[2])*sin(th[1])) + 0.31105*cos(th[0])*cos(th[1])*cos(th[2]),
            0.26375*cos(th[4])*(sin(th[0])*sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - 0.31105*sin(th[0])*sin(th[1])*sin(th[2]) + 0.26375*cos(th[3])*sin(th[4])*(cos(th[1])*sin(th[0])*sin(th[2]) - 1.0*cos(th[2])*sin(th[0])*sin(th[1])) - 0.31105*cos(th[1])*cos(th[2])*sin(th[0]),
            0.26375*cos(th[4])*(cos(th[1])*sin(th[2]) - 1.0*cos(th[2])*sin(th[1])) - 0.31105*cos(th[1])*sin(th[2]) + 0.31105*cos(th[2])*sin(th[1]) - 0.26375*cos(th[3])*sin(th[4])*(cos(th[1])*cos(th[2]) + sin(th[1])*sin(th[2]));


    Jv_col4 << -0.26375*sin(th[4])*(cos(th[3])*sin(th[0]) + sin(th[3])*(cos(th[0])*sin(th[1])*sin(th[2]) + cos(th[0])*cos(th[1])*cos(th[2]))),
               -0.26375*sin(th[4])*(cos(th[0])*cos(th[3]) - 1.0*sin(th[3])*(sin(th[0])*sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0]))),
                0.26375*sin(th[3])*sin(th[4])*(cos(th[1])*sin(th[2]) - 1.0*cos(th[2])*sin(th[1]));

    Jv_col5 << 0.26375*sin(th[4])*(cos(th[0])*cos(th[1])*sin(th[2]) - 1.0*cos(th[0])*cos(th[2])*sin(th[1])) - 0.26375*cos(th[4])*(sin(th[0])*sin(th[3]) - 1.0*cos(th[3])*(cos(th[0])*sin(th[1])*sin(th[2]) + cos(th[0])*cos(th[1])*cos(th[2]))),
             - 0.26375*cos(th[4])*(cos(th[0])*sin(th[3]) + cos(th[3])*(sin(th[0])*sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0]))) - 0.26375*sin(th[4])*(cos(th[1])*sin(th[0])*sin(th[2]) - 1.0*cos(th[2])*sin(th[0])*sin(th[1])),
             0.26375*sin(th[4])*(cos(th[1])*cos(th[2]) + sin(th[1])*sin(th[2])) - 0.26375*cos(th[3])*cos(th[4])*(cos(th[1])*sin(th[2]) - 1.0*cos(th[2])*sin(th[1]));

    Jv_col6 << 0.0,0.0,0.0;

    Jacobian_v << Jv_col1, Jv_col2,Jv_col3,Jv_col4,Jv_col5,Jv_col6;

    Jw_col1 << 0,0, -1;
    Jw_col2 << -sin(th[0]),-cos(th[0]), 0;
    Jw_col3 << sin(th[0]),cos(th[0]), 0;

    Jw_col4 << cos(th[0])*cos(th[2])*sin(th[1])-cos(th[0])*cos(th[1])*sin(th[2]),
           cos(th[1])*sin(th[0])*sin(th[2])-cos(th[2])*sin(th[0])*sin(th[1]),
           -cos(th[1])*cos(th[2])-sin(th[1])*sin(th[2]);

    Jw_col5 << -cos(th[3])*sin(th[0])-sin(th[3])*(cos(th[0])*sin(th[1])*sin(th[2])+cos(th[0])*cos(th[1])*cos(th[2])),
           sin(th[3])*(sin(th[0])*sin(th[1])*sin(th[2])+cos(th[1])*cos(th[2])*sin(th[0]))-cos(th[0])*cos(th[3]),
           sin(th[3])*(cos(th[1])*sin(th[2])-cos(th[2])*sin(th[1]));

    Jw_col6 << sin(th[4])*(sin(th[0])*sin(th[3]) - 1.0*cos(th[3])*(cos(th[0])*sin(th[1])*sin(th[2]) + cos(th[0])*cos(th[1])*cos(th[2]))) + cos(th[4])*(cos(th[0])*cos(th[1])*sin(th[2]) - 1.0*cos(th[0])*cos(th[2])*sin(th[1])),
            sin(th[4])*(cos(th[0])*sin(th[3]) + cos(th[3])*(sin(th[0])*sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0]))) - 1.0*cos(th[4])*(cos(th[1])*sin(th[0])*sin(th[2]) - 1.0*cos(th[2])*sin(th[0])*sin(th[1])),
            cos(th[4])*(cos(th[1])*cos(th[2]) + sin(th[1])*sin(th[2])) + cos(th[3])*sin(th[4])*(cos(th[1])*sin(th[2]) - 1.0*cos(th[2])*sin(th[1]));

    Jacobian_w << Jw_col1, Jw_col2,Jw_col3,Jw_col4,Jw_col5,Jw_col6;

    J << Jacobian_v, Jacobian_w;
    std::cout << "------Jacobian Matrix------" << std::endl;
    std::cout << J << std::endl;
}


