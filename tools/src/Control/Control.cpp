//
// Created by chansik on 23. 12. 11.
//
#include "Control/Control.hpp"



Eigen::VectorXd control::ComputeFK(Eigen::VectorXd &inputjoints){ ///void 말고 joint

    Eigen::MatrixXd result;
    Eigen::VectorXd FKvalue(6);

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

    getOrientation(result,FKvalue);

//    std::cout << result << std::endl;

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

void control::getOrientation(Eigen::MatrixXd Tmatrix, Eigen::VectorXd &FKvalue){

    Eigen::MatrixXd Orientation(3,3);
    Eigen::MatrixXd FindAngle(2,2);
    FindAngle.setZero();
    Orientation.setZero();
    Orientation << Tmatrix.block<3,3>(0,0);

    const double epsilon = 1e-3;
    double check_sign = -Orientation(2,0);
    double Roll, Pitch, Yaw;
    double E1,E2,E3,E4;

    if(abs(check_sign-1)< epsilon) { /// singularity
        Roll = atan2(Orientation(0,1),Orientation(1,1));
        Pitch = PI/2;
        Yaw = 0;
    }
    else if(abs(check_sign+1)< epsilon){ /// singularity
        Roll = -atan2(Orientation(0,1),Orientation(1,1));
        Pitch = -PI/2;
        Yaw = 0;
    }
    /// sinB >0,<0 ,,, B : -PI~PI
    else {
        ///pitch angle B : 0~PI/2 가정, cosB>0
        Roll = atan2(Orientation(2,1),Orientation(2,2));
        Pitch = atan2(-Orientation(2,0),sqrt(pow(Orientation(0,0),2)+pow(Orientation(1,0),2)));
        Yaw = atan2(Orientation(1,0),Orientation(0,0));

        FindAngle(0,0) = cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll);
        FindAngle(1,0) = sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll);
        FindAngle(0,1) = cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll);
        FindAngle(1,1) = sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll);

        E1 = abs(FindAngle(0,0)-Orientation(0,1));
        E2 = abs(FindAngle(1,0)-Orientation(1,1));
        E3 = abs(FindAngle(0,1)-Orientation(0,2));
        E4 = abs(FindAngle(1,1)-Orientation(1,2));

        if(E1<0.01 && E2<0.01 && E3<0.01 && E4<0.01){

        }
        else{
        ///pitch angle B PI/2 ~ PI => cosB <0
            Roll = atan2(-Orientation(2,1),-Orientation(2,2));
            Pitch = atan2(-Orientation(2,0),-sqrt(pow(Orientation(0,0),2)+pow(Orientation(1,0),2)));
            Yaw = atan2(-Orientation(1,0),-Orientation(0,0));
        }
    }
    ///pitch angle B : -PI~0
//    else if (check_sign < 0){
//        ///pitch angle B : -PI/2 ~ 0 가정, cosB >0
//        Roll = atan2(Orientation(2,2),Orientation(2,1));
//        Pitch = atan2(sqrt(pow(Orientation(0,0),2)+pow(Orientation(1,0),2)),-Orientation(2,0));
//        Yaw = atan2(Orientation(0,0),Orientation(1,0));
//
//        FindAngle(0,0) = cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll);
//        FindAngle(1,0) = sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll);
//        FindAngle(0,1) = cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll);
//        FindAngle(1,1) = sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll);
//
//        E1 = abs(FindAngle(0,0)-Orientation(0,1));
//        E2 = abs(FindAngle(1,0)-Orientation(1,1));
//        E3 = abs(FindAngle(0,1)-Orientation(0,2));
//        E4 = abs(FindAngle(1,1)-Orientation(1,2));
//
//
//        if(E1<0.01 && E2<0.01 && E3<0.01 && E4<0.01){
//
//        }
//        else{
//        ///pitch angle B -PI ~ -PI/2 => cosB <0
//            Roll = atan2(-Orientation(2,2),-Orientation(2,1));
//            Pitch = atan2(-sqrt(pow(Orientation(0,0),2)+pow(Orientation(1,0),2)),-Orientation(2,0));
//            Yaw = atan2(-Orientation(0,0),-Orientation(1,0));
//        }
//    }
    for(int i=0; i<3; i++){
        FKvalue[i] = (Tmatrix(i,3));
    }
    FKvalue[3] = Roll;
    FKvalue[4] = Pitch;
    FKvalue[5] = Yaw;
}

Eigen::VectorXd control::ComputeIK(Eigen::VectorXd &initial_angle, Eigen::VectorXd goal_pose,raisim::ArticulatedSystem *robot){

//    double epsilon = 0.000001;
//    double step_size = 0.005;
    double epsilon = 0.00001; /// 0.003       150 80 90 0 90 60 기준
    double step_size = 0.05;/// 0.02   현재 150 80 90 0 90 60 실험 중
    Eigen::VectorXd delta_X(robot->getDOF());
    Eigen::VectorXd Task_cur(robot->getDOF());
    Eigen::VectorXd Joint_cur(robot->getDOF());
    Eigen::VectorXd IK_TestVal(robot->getDOF());

    for(int i=3; i<6; i++){
        goal_pose[i] = d2r*goal_pose[i]; /// goal pose: degree -> radian, initial angle : radian
    }

    std::cout << "Task_cur : " << std::endl;
    Task_cur=ComputeFK(initial_angle);
    std::cout << Task_cur << std::endl;

    std::cout << "goal_pose : " << std::endl;
    std::cout << goal_pose << std::endl;

    delta_X << step_size*(goal_pose-Task_cur);
    std::cout << "delta_x " << std::endl;
    std::cout << delta_X << std::endl;

    LoopCount = 0;
    while((delta_X.array().abs().maxCoeff() > epsilon)){
        if(LoopCount == 0){
            std::cout << "Wait..." << std::endl;
            getJacobian(initial_angle);  ///update jacobian for every tik, initial angle : radian
            Inv_J = J.inverse();
            Joint_cur << initial_angle+Inv_J*delta_X;
            LoopCount++;
        }
        else{
            Task_cur = ComputeFK(Joint_cur);
            delta_X = step_size*(goal_pose-Task_cur);
//            std::cout << "Descending..." << std::endl;
            getJacobian(Joint_cur);  ///update jacobian for every tik, initial angle : radian
            Inv_J = J.inverse();
//            std::cout << Inv_J << std::endl;
            std::cout << "----------------------" << std::endl;
            std::cout << delta_X << std::endl;
            Joint_cur << Joint_cur+Inv_J*delta_X;
            LoopCount++;
//            if(LoopCount >= 50000)
//                break;
        }
    }
    IK_TestVal << ComputeFK(Joint_cur);
    IK_TestVal[3] = IK_TestVal[3]*r2d;
    IK_TestVal[4] = IK_TestVal[4]*r2d;
    IK_TestVal[5] = IK_TestVal[5]*r2d;

    std::cout << "While ended" << std::endl;
    std::cout << "IK checking!!! : computed Joints" << std::endl;
    std::cout << Joint_cur*180/PI << std::endl; ///degree
//    std::cout << "Check FK value using IK computed joints" << std::endl;
//    std::cout << IK_TestVal << std::endl;
    return Joint_cur;
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
//    std::cout << "------Jacobian Matrix------" << std::endl;
//    std::cout << J << std::endl;

    testJ << J;
}

void control::JointPDControl(Eigen::VectorXd IKJoints, raisim::World* world, raisim::ArticulatedSystem *robot, float timeDuration){
    cubicTrajectoryGenerator trajectoryGenerator[robot->getDOF()];
    setTime setTime;

    mPgain << 80.0, 80.0, 80.0, 40.0, 40.0, 40.0;
    mDgain << 2.0, 2.0, 2.0, 0.5, 0.5, 0.5;
    Eigen::VectorXd Reference_pose(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd Force(robot->getDOF());
    Force <<25, 25, 25, 25, 25,25;
//    Force.setZero();
    /// get joint current state
    for (int i = 0; i < robot->getDOF(); i++)
    {
        currentPosition(i) = robot->getGeneralizedCoordinate()[i] ;
    }

    /// set time
    setTime.setTimeInitiallize();
    setTime.timedT = 0.01;

    /// set joint goal position
    std::cout << " " << std::endl;
    Reference_pose = IKJoints; ///radian

    /// check goal position
    std::cout << "Reference_pose  :  ";
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        std::cout << Reference_pose[i] << " ";
    }

    /// create trajectory
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],Reference_pose[i],setTime.localtime,timeDuration)  ;
    } ///calculate the Coefficient for cubic

    while (1)
    {
        setTime.setLocaltime(); //get in while loop. time step increasing
        for (int jointNum = 0; jointNum < robot->getGeneralizedCoordinateDim() ; jointNum++)
        {
            jointPositionTarget[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
            jointVelocityTarget[jointNum] = trajectoryGenerator[jointNum].getVelocityTrajectory(setTime.localtime);
        }

        /// robot set position
        robot->setGeneralizedCoordinate(jointPositionTarget);
        robot->setGeneralizedForce(Force);
        world->integrate();
        usleep(5000);
        if (setTime.localtime >= timeDuration)
            break;
    }

    robot->setPdGains(mPgain, mDgain);
    robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
//    Force.setZero();
//    robot->setGeneralizedForce(Force);

//    for(int i=0; i<6; i++){
//        std::cout << JointSpaceInput[i] << " ";
//    }
    std::cout << std::endl;
//    std::cout << "world time check : " << world->getWorldTime() << std::endl;

    std::cout << "\n" <<  "robot current position  :  " << robot->getGeneralizedCoordinate() << std::endl;
}




