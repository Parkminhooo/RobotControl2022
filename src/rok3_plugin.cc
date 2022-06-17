/*
 * RoK-3 Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer : Yunho Han
 * Second developer : Minho Park
 * #include <iostream>

 * ======
 * Update date : 2022.03.16 by Yunho Han
 * ======
 */
//* Header file for C++
#include <stdio.h>
#include <boost/bind.hpp>

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

//RBDL//
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;

double deg2rad = PI / 180;
double rad2deg = 180 / PI;

Vector3d r_des, l_r_des, r_r_des;
MatrixXd C_des;
Matrix3d L_C_des, L_C_predes, R_C_predes, R_C_des;
VectorXd q_cal(12), q_init(6), l_q_cal(6), r_q_cal(6), l_q_init(6), r_q_init(6);


//// init des_q
Eigen::VectorXd des_q(6);


namespace gazebo
{

    class rok3_plugin : public ModelPlugin
    {
        //*** Variables for RoK-3 Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //* Model & Link & Joint Typedefs
        physics::ModelPtr model;

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        physics::JointPtr torso_joint;

        physics::JointPtr LS, RS;

        //* Index setting for each joint

        enum
        {
            WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;

        //*ROS publisher
        ros::Publisher L_R_DES_Z;

        std_msgs::Float32MultiArray l_r_des_z;

        ros::NodeHandle nh;




    public:
        //*** Functions for RoK-3 Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointController(); // Joint Controller for each joint
        void ROSMsgPublish();

        void GetJoints(); // Get each joint data from [physics::ModelPtr _model]
        void GetjointData(); // Get encoder data of each joint

        void initializeJoint(); // Initialize joint variables for joint control
        void SetJointPIDgain(); // Set each joint PID gain for joint control
    };
    GZ_REGISTER_MODEL_PLUGIN(rok3_plugin);
}
//* getTransformIo()

MatrixXd getTransformI0()
{
    MatrixXd tmp_m(4, 4);


    tmp_m = MatrixXd::Identity(4, 4);

    /*
     tmp_m << 1, 0, 0, 0, \
     *        0, 1, 0, 0, \
     *        0, 0, 1, 0, \
     *        0, 0, 0, 1;
     */

    /*
     tmp_m(0, 0) = 1; tmp(0, 1) = 0; ...
     * ...
     * ...
     */


    return tmp_m;
}

MatrixXd jointToTransform01(VectorXd q, std::string dir = "left") // Hip Yaw
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4, 4);
    double tmp_q = q(0);

    std::string L_R_step = dir;
    if (L_R_step == "left") {
        tmp_m << cos(tmp_q), -sin(tmp_q), 0, 0, \
             sin(tmp_q), cos(tmp_q), 0, 0.105, \
             0, 0, 1, -0.1512, \
             0, 0, 0, 1;
    }
    else if (L_R_step == "right") {
        tmp_m << cos(tmp_q), -sin(tmp_q), 0, 0, \
             sin(tmp_q), cos(tmp_q), 0, -0.105, \
             0, 0, 1, -0.1512, \
             0, 0, 0, 1;
    }

    return tmp_m;
}

MatrixXd jointToTransform12(VectorXd q) // Hip Roll
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4, 4);
    double tmp_q = q(1);

    tmp_m << 1, 0, 0, 0, \
             0, cos(tmp_q), -sin(tmp_q), 0, \
             0, sin(tmp_q), cos(tmp_q), 0, \
             0, 0, 0, 1;

    return tmp_m;
}

MatrixXd jointToTransform23(VectorXd q) // Hip Pitch
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4, 4);
    double tmp_q = q(2);

    tmp_m << cos(tmp_q), 0, sin(tmp_q), 0, \
             0, 1, 0, 0, \
             -sin(tmp_q), 0, cos(tmp_q), 0, \
             0, 0, 0, 1;

    return tmp_m;
}

MatrixXd jointToTransform34(VectorXd q) // Knee Pitch
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4, 4);
    double tmp_q = q(3);

    tmp_m << cos(tmp_q), 0, sin(tmp_q), 0, \
             0, 1, 0, 0, \
             -sin(tmp_q), 0, cos(tmp_q), -0.35, \
             0, 0, 0, 1;



    return tmp_m;
}

MatrixXd jointToTransform45(VectorXd q) // Ankle Pitch
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4, 4);
    double tmp_q = q(4);

    tmp_m << cos(tmp_q), 0, sin(tmp_q), 0, \
             0, 1, 0, 0, \
             -sin(tmp_q), 0, cos(tmp_q), -0.35, \
             0, 0, 0, 1;



    return tmp_m;
}

MatrixXd jointToTransform56(VectorXd q) // Ankle Roll
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4, 4);
    double tmp_q = q(5);

    tmp_m << 1, 0, 0, 0, \
             0, cos(tmp_q), -sin(tmp_q), 0, \
             0, sin(tmp_q), cos(tmp_q), 0, \
             0, 0, 0, 1;



    return tmp_m;
}

MatrixXd getTransform6E() // Ankle Roll
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4, 4);

    tmp_m << 1, 0, 0, 0, \
             0, 1, 0, 0, \
             0, 0, 1, -0.09, \
             0, 0, 0, 1;



    return tmp_m;
}

VectorXd jointToPosition(VectorXd q, std::string dir = "left")
{
    MatrixXd tmp_m(4, 4);
    Vector3d tmp_p;

    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);
    std::string L_R_step = dir;

    T_I0 = getTransformI0();
    if (L_R_step == "left") {
        T_01 = jointToTransform01(q);
    }
    else if (L_R_step == "right") {
        T_01 = jointToTransform01(q, "right");
    }
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    tmp_m = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56*T_6E;

    tmp_p = tmp_m.block(0, 3, 3, 1);

    return tmp_p;


}

MatrixXd jointToRotMat(VectorXd q, std::string dir = "left")
{
    MatrixXd tmp_m(4, 4);
    MatrixXd tmp_return(3, 3);

    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);

    std::string L_R_step = dir;
    T_I0 = getTransformI0();
    if (L_R_step == "left") {
        T_01 = jointToTransform01(q);
    }
    else if (L_R_step == "right") {
        T_01 = jointToTransform01(q, "right");
    }
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    tmp_m = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56*T_6E;

    tmp_return = tmp_m.block(0, 0, 3, 3);

    return tmp_return;
}

VectorXd rotToEuler(MatrixXd rotMat)
{
    double z = atan2(rotMat(1, 0), rotMat(0, 0));
    double y = atan2(-rotMat(2, 0), sqrt(rotMat(2, 1) * rotMat(2, 1) + rotMat(2, 2) * rotMat(2, 2)));
    double x = atan2(rotMat(2, 1), rotMat(2, 2));

    Vector3d tmp_v(z, y, x);
    return tmp_v;
}

double func_1_cos(double t, double init, double final_, double T)
{
    double des;
    double omega = PI / T;
    des = (final_ - init) * 0.5 * (1 - cos(omega * t)) + init;

    return des;
}

double cosWave(double amp, double period, double time, double init_pos)
{
    //amp : (final - init) value
    //period : time period
    //time : current time
    //init_pos : init value
    return (amp - init_pos) *0.5 * (1 - cos(PI / period * time)) + init_pos;
}

MatrixXd jointToPosJac(VectorXd q, std::string dir = "left")
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3, 6);
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4), T_IE(4, 4);
    MatrixXd T_I1(4, 4), T_I2(4, 4), T_I3(4, 4), T_I4(4, 4), T_I5(4, 4), T_I6(4, 4);
    MatrixXd R_I1(3, 3), R_I2(3, 3), R_I3(3, 3), R_I4(3, 3), R_I5(3, 3), R_I6(3, 3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1, n_I_2, n_I_3, n_I_4, n_I_5, n_I_6;
    Vector3d r_I_IE;

    std::string L_R_step = dir;
    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    if (L_R_step == "left") {
        T_01 = jointToTransform01(q);
    }
    else if (L_R_step == "right") {
        T_01 = jointToTransform01(q, "right");
    }
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();
    T_IE = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56*T_6E;

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0 * T_01;
    T_I2 = T_I1 * T_12;
    T_I3 = T_I2 * T_23;
    T_I4 = T_I3 * T_34;
    T_I5 = T_I4 * T_45;
    T_I6 = T_I5 * T_56;

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0, 0, 3, 3);
    R_I2 = T_I2.block(0, 0, 3, 3);
    R_I3 = T_I3.block(0, 0, 3, 3);
    R_I4 = T_I4.block(0, 0, 3, 3);
    R_I5 = T_I5.block(0, 0, 3, 3);
    R_I6 = T_I6.block(0, 0, 3, 3);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0, 3, 3, 1);
    r_I_I2 = T_I2.block(0, 3, 3, 1);
    r_I_I3 = T_I3.block(0, 3, 3, 1);
    r_I_I4 = T_I4.block(0, 3, 3, 1);
    r_I_I5 = T_I5.block(0, 3, 3, 1);
    r_I_I6 = T_I6.block(0, 3, 3, 1);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0, 0, 1; //Hip Yaw
    n_2 << 1, 0, 0; //Hip Roll
    n_3 << 0, 1, 0; //Hip Pitch
    n_4 << 0, 1, 0; //Knee Pitch
    n_5 << 0, 1, 0; //Ankle Pitch
    n_6 << 1, 0, 0; //Ankle Roll

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the end-effector position vector.
    r_I_IE = T_IE.block(0, 3, 3, 1);

    //* Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE - r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE - r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE - r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE - r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE - r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE - r_I_I6);

    //    std::cout << "Test, J_P:" << std::endl << J_P << std::endl;

    return J_P;
}

MatrixXd jointToRotJac(VectorXd q, std::string dir = "left")
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3, 6);
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);
    MatrixXd T_I1(4, 4), T_I2(4, 4), T_I3(4, 4), T_I4(4, 4), T_I5(4, 4), T_I6(4, 4);
    MatrixXd R_I1(3, 3), R_I2(3, 3), R_I3(3, 3), R_I4(3, 3), R_I5(3, 3), R_I6(3, 3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;

    std::string L_R_step = dir;
    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    if (L_R_step == "left") {
        T_01 = jointToTransform01(q);
    }
    else if (L_R_step == "right") {
        T_01 = jointToTransform01(q);
    }
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0 * T_01;
    T_I2 = T_I1 * T_12;
    T_I3 = T_I2 * T_23;
    T_I4 = T_I3 * T_34;
    T_I5 = T_I4 * T_45;
    T_I6 = T_I5 * T_56;

    //* Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block(0, 0, 3, 3);
    R_I2 = T_I2.block(0, 0, 3, 3);
    R_I3 = T_I3.block(0, 0, 3, 3);
    R_I4 = T_I4.block(0, 0, 3, 3);
    R_I5 = T_I5.block(0, 0, 3, 3);
    R_I6 = T_I6.block(0, 0, 3, 3);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0, 0, 1; //Hip Yaw
    n_2 << 1, 0, 0; //Hip Roll
    n_3 << 0, 1, 0; //Hip Pitch
    n_4 << 0, 1, 0; //Knee Pitch
    n_5 << 0, 1, 0; //Ankle Pitch
    n_6 << 1, 0, 0; //Ankle Roll

    //* Compute the translational Jacobian.
    J_R.col(0) << R_I1*n_1;
    J_R.col(1) << R_I2*n_2;
    J_R.col(2) << R_I3*n_3;
    J_R.col(3) << R_I4*n_4;
    J_R.col(4) << R_I5*n_5;
    J_R.col(5) << R_I6*n_6;

    //    std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

MatrixXd jointToJacobian(VectorXd q, std::string dir = "left")
{
    MatrixXd J = MatrixXd::Zero(6, 6);
    std::string L_R_step = dir;
    if (L_R_step == "left") {
        J << jointToPosJac(q),
                jointToRotJac(q);
    }
    else if (L_R_step == "right") {
        J << jointToPosJac(q, "right"),
                jointToRotJac(q, "right");
    }
    std::cout << "geometric Jacobian : " << J << std::endl;
    return J;
}

MatrixXd pseudoInverseMat(MatrixXd A, double lambda)
{
    // Input: Any m-by-n matrix
    // Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    static int m = A.rows();
    static int n = A.cols();
    MatrixXd pinvA;
    MatrixXd tmpA;

    if (m >= n) {
        //left pseudo-inverse
        //        std::cout<<"m: "<< m << std::endl;
        //        std::cout<<"n: "<< n << std::endl;
        //        std::cout << "right damped pseudo-inverse" << std::endl;
        tmpA = A.transpose() * A + lambda * lambda * MatrixXd::Identity(n, n);
        pinvA = tmpA.inverse() * A.transpose();
        //        std::cout << "pinv : " << pinvA << std::endl;
    }

    else if (m < n) {
        //right pseudo-inverse
        //        std::cout<<"m: "<< m << std::endl;
        //        std::cout<<"n: "<< n << std::endl;
        //        std::cout << "right damped pseudo-inverse" << std::endl;

        tmpA = A * A.transpose() + lambda * lambda * MatrixXd::Identity(m, m);
        pinvA = A.transpose() * tmpA.inverse();
        //        std::cout << "pinv : " << pinvA << std::endl;
    }

    return pinvA;
}

VectorXd rotMatToRotVec(MatrixXd C)
{
    // Input : a rotation matrix C
    // Output : the rotational vector which describes the rotation C
    Vector3d phi, n;
    double th;

    th = acos((C(0, 0) + C(1, 1) + C(2, 2) - 1) / 2);

    if (fabs(th) < 0.001) {
        //singularity
        n << 0, 0, 0;
    }
    else {
        n << 1 / (2 * sin(th)) * (C(2, 1) - C(1, 2)),
                1 / (2 * sin(th)) * (C(0, 2) - C(2, 0)),
                1 / (2 * sin(th)) * (C(1, 0) - C(0, 1));

    }

    phi = th*n;

    return phi;
}
//
//VectorXd inverseKinematics(Vector3d L_r_des, MatrixXd L_C_des, Vector3d R_r_des, MatrixXd R_C_des, VectorXd L_q0, VectorXd R_q0, double tol)
//{
//    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
//    // Output: joint angles which match desired end-effector position and orientation
//    double num_it;
//    MatrixXd J_P(3, 6), J_R(3, 6), J(6, 6), pinvJ(6, 6), C_err(3, 3), C_IE(3, 3);
//    MatrixXd L_J_P(3, 6), L_J_R(3, 6), L_J(6, 6), L_pinvJ(6, 6), L_C_err(3, 3), L_C_IE(3, 3);
//    MatrixXd R_J_P(3, 6), R_J_R(3, 6), R_J(6, 6), R_pinvJ(6, 6), R_C_err(3, 3), R_C_IE(3, 3);
//    VectorXd q(12), dq(6), dXe(6), L_q(6), R_q(6), L_dq(6), R_dq(6), L_dXe(6), R_dXe(6);
//    Vector3d dr, dph, L_dr, L_dph, R_dr, R_dph;
//    double lambda;
//
//    //* Set maximum number of iterations
//    double max_it = 200;
//
//    //* Initialize the solution with the initial guess
//    L_q = L_q0;
//    R_q = R_q0;
//
//
//    L_C_IE = jointToRotMat(L_q);
//    R_C_IE = jointToRotMat(R_q, "right");
//
//    L_C_err = L_C_des * L_C_IE.transpose();
//    R_C_err = R_C_des * R_C_IE.transpose();
//
//    //* Damping factor
//    lambda = 0.001;
//
//    //* Initialize error
//    L_dr = L_r_des - jointToPosition(L_q);
//    R_dr = R_r_des - jointToPosition(R_q, "right");
//
//    L_dph = rotMatToRotVec(L_C_err);
//    R_dph = rotMatToRotVec(R_C_err);
//
//    L_dXe << L_dr(0), L_dr(1), L_dr(2), L_dph(0), L_dph(1), L_dph(2);
//    R_dXe << R_dr(0), R_dr(1), R_dr(2), R_dph(0), R_dph(1), R_dph(2);
//
//    ////////////////////////////////////////////////
//    //** Iterative inverse kinematics
//    ////////////////////////////////////////////////
//
//    //* Iterate until terminating condition
//    while ((num_it < max_it) && (L_dXe.norm() > tol) && (R_dXe.norm() > tol)) {
//
//        //Compute Inverse Jacobian
//        L_J_P = jointToPosJac(L_q);
//        L_J_R = jointToRotJac(L_q);
//
//        R_J_P = jointToPosJac(R_q, "right");
//        R_J_R = jointToRotJac(R_q, "right");
//
//        L_J.block(0, 0, 3, 6) = L_J_P;
//        L_J.block(3, 0, 3, 6) = L_J_R; // Geometric Jacobian
//
//        R_J.block(0, 0, 3, 6) = R_J_P;
//        R_J.block(3, 0, 3, 6) = R_J_R; // Geometric Jacobian
//
//        // Convert to Geometric Jacobian to Analytic Jacobian
//        L_dq = pseudoInverseMat(L_J, lambda) * L_dXe;
//        R_dq = pseudoInverseMat(R_J, lambda) * R_dXe;
//
//        // Update law
//        L_q += 0.5 * L_dq;
//        R_q += 0.5 * R_dq;
//
//        // Update error
//        L_C_IE = jointToRotMat(L_q);
//        L_C_err = L_C_des * L_C_IE.transpose();
//
//        L_dr = L_r_des - jointToPosition(L_q);
//        L_dph = rotMatToRotVec(L_C_err);
//        L_dXe << L_dr(0), L_dr(1), L_dr(2), L_dph(0), L_dph(1), L_dph(2);
//
//        R_C_IE = jointToRotMat(R_q, "right");
//        R_C_err = R_C_des * R_C_IE.transpose();
//
//        R_dr = R_r_des - jointToPosition(R_q, "right");
//        R_dph = rotMatToRotVec(R_C_err);
//        R_dXe << R_dr(0), R_dr(1), R_dr(2), R_dph(0), R_dph(1), R_dph(2);
//
//        num_it++;
//        q << L_q, R_q;
//    }
//    //        std::cout << "iteration: " << num_it << std::endl << ", value (deg) : " << q * rad2deg << std::endl;
//
//    return q;
//}

VectorXd inverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol, std::string dir = "left")
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation
    double num_it;
    MatrixXd J_P(3, 6), J_R(3, 6), J(6, 6), pinvJ(6, 6), C_err(3, 3), C_IE(3, 3);
    VectorXd q(6), dq(6), dXe(6);
    Vector3d dr, dph;
    double lambda;
    std::string L_R_step = dir;

    //* Set maximum number of iterations
    double max_it = 200;

    //* Initialize the solution with the initial guess
    q = q0;
    if (L_R_step == "left") {
        C_IE = jointToRotMat(q);
    }
    else if (L_R_step == "right") {
        C_IE = jointToRotMat(q, "right");
    }
    C_err = C_des * C_IE.transpose();

    //* Damping factor
    lambda = 0.001;

    //* Initialize error
    if (L_R_step == "left") {
        dr = r_des - jointToPosition(q);
    }
    else if (L_R_step == "right") {
        dr = r_des - jointToPosition(q, "right");
    }
    dph = rotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);

    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////

    //* Iterate until terminating condition
    while (num_it < max_it && dXe.norm() > tol) {

        //Compute Inverse Jacobian
        if (L_R_step == "left") {
            J_P = jointToPosJac(q);
            J_R = jointToRotJac(q);
        }
        else if (L_R_step == "right") {
            J_P = jointToPosJac(q, "right");
            J_R = jointToRotJac(q, "right");
        }


        J.block(0, 0, 3, 6) = J_P;
        J.block(3, 0, 3, 6) = J_R; // Geometric Jacobian

        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J, lambda) * dXe;

        // Update law
        q += 0.5 * dq;

        // Update error
        if (L_R_step == "left") {
            C_IE = jointToRotMat(q);
            C_err = C_des * C_IE.transpose();

            dr = r_des - jointToPosition(q);
            dph = rotMatToRotVec(C_err);
            dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
        }
        else if (L_R_step == "right") {
            C_IE = jointToRotMat(q, "right");
            C_err = C_des * C_IE.transpose();

            dr = r_des - jointToPosition(q, "right");
            dph = rotMatToRotVec(C_err);
            dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
        }

        num_it++;
    }
    //    std::cout << "iteration: " << num_it << std::endl; // << ", value (deg) : " << q * rad2deg << std::endl;

    return q;
}

//* Preparing RobotControl Practice

void Practice()
{
    Vector3d r_des;
    MatrixXd C_des;
    VectorXd q(6), q_cal(6), q_init(6);
    q << 10 * deg2rad, 20 * deg2rad, 30 * deg2rad, 40 * deg2rad, 50 * deg2rad, 60 * deg2rad;
    q_init << 0 * deg2rad, 0 * deg2rad, -30 * deg2rad, 60 * deg2rad, -30 * deg2rad, 0 * deg2rad;

    // q = [10;20;30;40;50;60]*pi/180;
    r_des << 0, 0.105, -0.55;
    C_des = MatrixXd::Identity(3, 3);

    //    q_cal = inverseKinematics(r_des, C_des, q_init, 0.001);
    des_q = q_cal;
    std::cout << "q_cal : " << q_cal * rad2deg << std::endl;
}

void gazebo::rok3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */

    L_R_DES_Z = nh.advertise<std_msgs::Float32MultiArray>("l_r_des_z", 1000);
    l_r_des_z.data.resize(1);


    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* rok3_model] for using RBDL
    Model* rok3_model = new Model();
    Addons::URDFReadFromFile("/home/minho/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    initializeJoint();
    SetJointPIDgain();


    //* setting for getting dt
    last_update_time = model->GetWorld()->GetSimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));


    //    Practice();

    //    jointToPosJac(q);
    //    jointToRotJac(q);

    //    jointToJacobian(q);
    //    

    //    VectorXd q(6);
    //    q << 10 * deg2rad, 20 * deg2rad, 30 * deg2rad, 40 * deg2rad, 50 * deg2rad, 60 * deg2rad;
    //    std::cout << "pseudo : " << pseudoInverseMat(jointToJacobian(q), 0.001)  << std::endl;
    //    
    //    MatrixXd C_des  = jointToRotMat(q);
    //    MatrixXd C_init = jointToRotMat(q_init);
    //    
    //    MatrixXd C_err = C_des*C_init.transpose();
    //    
    //    VectorXd dph = rotMatToRotVec(C_err);

    //    std::cout << "dph : " << dph << std::endl;
}

void gazebo::rok3_plugin::UpdateAlgorithm()
{
    /*
     * Algorithm update while simulation
     */

    //* UPDATE TIME : 1ms
    common::Time current_time = model->GetWorld()->GetSimTime();
    dt = current_time.Double() - last_update_time.Double();
    //    cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;


    //* Read Sensors data
    GetjointData();

    //Walk Ready
    if (time <= 5.) {
        if (time == 0.001) {
            q_init << 0 * deg2rad, 0 * deg2rad, -30 * deg2rad, 60 * deg2rad, -30 * deg2rad, 0 * deg2rad;
            // q = [10;20;30;40;50;60]*pi/180;
            l_r_des << 0, 0.105, -0.75;
            r_r_des << 0, -0.105, -0.75;
            C_des = MatrixXd::Identity(3, 3);

            l_q_cal = inverseKinematics(l_r_des, C_des, q_init, 0.001);
            r_q_cal = inverseKinematics(r_r_des, C_des, q_init, 0.001, "right");

            std::cout << "Walk Ready Angle" << std::endl;
            std::cout << "l_q_cal : " << l_q_cal << std::endl;
            std::cout << "r_q_cal : " << r_q_cal << std::endl;
        }
    }
        //Front Walk 3 steps.
        //Move com right
    else if (time <= 6) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(1) = cosWave(0.21, 1, time_, 0.105);
        r_r_des(1) = cosWave(0, 1, time_, -0.105);

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;

        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;

    }

        //Swing left leg
    else if (time <= 8.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = cosWave(0.2, 2.5, time_, 0);
        l_r_des(2) = cosWave(-0.65, 1.25, time_, -0.75);

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;

        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;

    }

        //Move CoM front, left
    else if (time <= 11) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = cosWave(0, 2.5, time_, 0.2);
        l_r_des(1) = cosWave(0, 2.5, time_, 0.21);
        l_r_des(2) = -0.75;

        r_r_des(0) = cosWave(-0.2, 2.5, time_, 0);
        r_r_des(1) = cosWave(-0.21, 2.5, time_, 0);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;

        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;

    }

        //Swing right leg
    else if (time <= 13.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.02, 2.5, time_, 0);
        l_r_des(2) = -0.75;

        r_r_des(0) = cosWave(0.2, 2.5, time_, -0.2);
        r_r_des(1) = cosWave(-0.19, 2.5, time_, -0.21);
        r_r_des(2) = cosWave(-0.65, 1.25, time_, -0.75);

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;

    }

        //Move CoM front,right
    else if (time <= 16) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = cosWave(-0.2, 2.5, time_, 0);
        l_r_des(1) = cosWave(0.21, 2.5, time_, 0.02);
        l_r_des(2) = -0.75;

        r_r_des(0) = cosWave(0, 2.5, time_, 0.2);
        r_r_des(1) = cosWave(0, 2.5, time_, -0.19);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //Swing left leg
    else if (time <= 18.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = cosWave(0, 2.5, time_, -0.2);
        l_r_des(1) = 0.21;
        l_r_des(2) = cosWave(-0.65, 1.25, time_, -0.75);

        r_r_des(0) = 0;
        r_r_des(1) = 0;
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //Move CoM center
    else if (time <= 19.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.105, 1, time_, 0.21);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(-0.105, 1, time_, 0);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //Turn 90 deg
        //First turn
        //Move CoM right
    else if (time <= 20.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.21, 1, time_, 0.105);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(0, 1, time_, -0.105);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //left leg 30 deg turn
    else if (time <= 23) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = 0.21;
        l_r_des(2) = cosWave(-0.65, 1.25, time_, -0.75);

        r_r_des(0) = 0;
        r_r_des(1) = 0;
        r_r_des(2) = -0.75;

        L_C_predes << cosWave(cos(30 * deg2rad), 2.5, time_, cos(0)), cosWave(-sin(30 * deg2rad), 2.5, time_, -sin(0)), 0, \
                                cosWave(sin(30 * deg2rad), 2.5, time_, sin(0)), cosWave(cos(30 * deg2rad), 2.5, time_, cos(0)), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //Move CoM left
    else if (time <= 24) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0, 1, time_, 0.21);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(-0.21, 1, time_, 0);
        r_r_des(2) = -0.75;

        L_C_predes << cos(30 * deg2rad), -sin(30 * deg2rad), 0, \
                                sin(30 * deg2rad), cos(30 * deg2rad), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //Swing right leg
    else if (time <= 26.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(-0.05, 2.5, time_, 0);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(-0.26, 2.5, time_, -0.21);
        r_r_des(2) = cosWave(-0.65, 2.5, time_, -0.75);

        L_C_predes << cosWave(cos(0 * deg2rad), 2.5, time_, cos(30 * deg2rad)), cosWave(-sin(0 * deg2rad), 2.5, time_, -sin(30 * deg2rad)), 0, \
                                cosWave(sin(0 * deg2rad), 2.5, time_, sin(30 * deg2rad)), cosWave(cos(0 * deg2rad), 2.5, time_, cos(30 * deg2rad)), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //right foot down
    else if (time <= 27.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = -0.05;
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = -0.26;
        r_r_des(2) = cosWave(-0.75, 1, time_, -0.65);

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //Second Turn (same move as the first motion.)
    else if (time <= 30) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.21, 2.5, time_, -0.05);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(0, 2.5, time_, -0.26);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 32.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.26, 2.5, time_, 0.21);
        l_r_des(2) = cosWave(-0.65, 1.25, time_, -0.75);

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(0.05, 2.5, time_, 0);
        r_r_des(2) = -0.75;

        L_C_predes << cosWave(cos(30 * deg2rad), 2.5, time_, cos(0)), cosWave(-sin(30 * deg2rad), 2.5, time_, -sin(0)), 0, \
                                cosWave(sin(30 * deg2rad), 2.5, time_, sin(0)), cosWave(cos(30 * deg2rad), 2.5, time_, cos(0)), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 35) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0, 2.5, time_, 0.26);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(-0.21, 2.5, time_, 0.05);
        r_r_des(2) = -0.75;

        L_C_predes << cos(30 * deg2rad), -sin(30 * deg2rad), 0, \
                                sin(30 * deg2rad), cos(30 * deg2rad), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 37.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(-0.05, 2.5, time_, 0);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(-0.26, 2.5, time_, -0.21);
        r_r_des(2) = cosWave(-0.65, 2.5, time_, -0.75);

        L_C_predes << cosWave(cos(0 * deg2rad), 2.5, time_, cos(30 * deg2rad)), cosWave(-sin(0 * deg2rad), 2.5, time_, -sin(30 * deg2rad)), 0, \
                                cosWave(sin(0 * deg2rad), 2.5, time_, sin(30 * deg2rad)), cosWave(cos(0 * deg2rad), 2.5, time_, cos(30 * deg2rad)), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 40) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = -0.05;
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = -0.26;
        r_r_des(2) = cosWave(-0.75, 2.5, time_, -0.65);

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //Third Turn (same move as the first motion.)
    else if (time <= 42.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.21, 2.5, time_, -0.05);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(0, 2.5, time_, -0.26);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 45) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.26, 2.5, time_, 0.21);
        l_r_des(2) = cosWave(-0.65, 1.25, time_, -0.75);

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(0.05, 2.5, time_, 0);
        r_r_des(2) = -0.75;

        L_C_predes << cosWave(cos(30 * deg2rad), 2.5, time_, cos(0)), cosWave(-sin(30 * deg2rad), 2.5, time_, -sin(0)), 0, \
                                cosWave(sin(30 * deg2rad), 2.5, time_, sin(0)), cosWave(cos(30 * deg2rad), 2.5, time_, cos(0)), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 47.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0, 2.5, time_, 0.26);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(-0.21, 2.5, time_, 0.05);
        r_r_des(2) = -0.75;

        L_C_predes << cos(30 * deg2rad), -sin(30 * deg2rad), 0, \
                                sin(30 * deg2rad), cos(30 * deg2rad), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 50) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(-0.05, 2.5, time_, 0);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(-0.26, 2.5, time_, -0.21);
        r_r_des(2) = cosWave(-0.65, 2.5, time_, -0.75);

        L_C_predes << cosWave(cos(0 * deg2rad), 2.5, time_, cos(30 * deg2rad)), cosWave(-sin(0 * deg2rad), 2.5, time_, -sin(30 * deg2rad)), 0, \
                                cosWave(sin(0 * deg2rad), 2.5, time_, sin(30 * deg2rad)), cosWave(cos(0 * deg2rad), 2.5, time_, cos(30 * deg2rad)), 0, \
                                0, 0, 1;

        l_q_cal = inverseKinematics(l_r_des, L_C_predes, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 52.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = -0.05;
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = -0.26;
        r_r_des(2) = cosWave(-0.75, 2.5, time_, -0.65);

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

        //Finished Turn
        //Second Walking start
    else if (time <= 55) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.21, 2.5, time_, -0.05);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(0, 2.5, time_, -0.26);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 57.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = cosWave(0.2, 2.5, time_, 0);
        l_r_des(1) = 0.21;
        l_r_des(2) = cosWave(-0.65, 1.25, time_, -0.75);

        r_r_des(0) = 0;
        r_r_des(1) = 0;
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 60) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = cosWave(0, 2.5, time_, 0.2);
        l_r_des(1) = cosWave(0, 2.5, time_, 0.21);
        l_r_des(2) = -0.75;

        r_r_des(0) = cosWave(-0.2, 2.5, time_, 0);
        r_r_des(1) = cosWave(-0.21, 2.5, time_, 0);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 62.5) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = 0;
        l_r_des(2) = -0.75;

        r_r_des(0) = cosWave(0, 2.5, time_, -0.2);
        r_r_des(1) = -0.21;
        r_r_des(2) = cosWave(-0.65, 1.25, time_, -0.75);

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else if (time <= 65) {
        static double time_ = 0;
        l_q_init = l_q_cal;
        r_q_init = r_q_cal;

        l_r_des(0) = 0;
        l_r_des(1) = cosWave(0.105, 2.5, time_, 0);
        l_r_des(2) = -0.75;

        r_r_des(0) = 0;
        r_r_des(1) = cosWave(-0.105, 2.5, time_, -0.21);
        r_r_des(2) = -0.75;

        l_q_cal = inverseKinematics(l_r_des, C_des, l_q_init, 0.001);
        r_q_cal = inverseKinematics(r_r_des, C_des, r_q_init, 0.001, "right");
        time_ = time_ + 0.001;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        cout.precision(7);
        std::cout << "L_Foot(X) : " << l_r_des(0) << " L_Foot(Y) : " << l_r_des(1) << " L_Foot(Z) : " << l_r_des(2) << std::endl;
        std::cout << "R_Foot(X) : " << r_r_des(0) << " R_Foot(Y) : " << r_r_des(1) << " R_Foot(Z) : " << r_r_des(2) << std::endl << std::endl;
        std::cout << "l_q_cal : " << std::endl << l_q_cal * rad2deg << std::endl;
        std::cout << "r_q_cal : " << std::endl << r_q_cal * rad2deg << std::endl;
    }

    else {
        printf(C_GREEN "Walking Finish \n" C_RESET);
    }
    //practice 6 - 1
    // joint 0 to q_Cal to 0 each  for 5 sec

    if (time <= 5) {
        joint[LHY].targetRadian = cosWave(l_q_cal(0), 5, time, 0);
        joint[LHR].targetRadian = cosWave(l_q_cal(1), 5, time, 0);
        joint[LHP].targetRadian = cosWave(l_q_cal(2), 5, time, 0);
        joint[LKN].targetRadian = cosWave(l_q_cal(3), 5, time, 0);
        joint[LAP].targetRadian = cosWave(l_q_cal(4), 5, time, 0);
        joint[LAR].targetRadian = cosWave(l_q_cal(5), 5, time, 0);

        joint[RHY].targetRadian = cosWave(r_q_cal(0), 5, time, 0);
        joint[RHR].targetRadian = cosWave(r_q_cal(1), 5, time, 0);
        joint[RHP].targetRadian = cosWave(r_q_cal(2), 5, time, 0);
        joint[RKN].targetRadian = cosWave(r_q_cal(3), 5, time, 0);
        joint[RAP].targetRadian = cosWave(r_q_cal(4), 5, time, 0);
        joint[RAR].targetRadian = cosWave(r_q_cal(5), 5, time, 0);
    }

    else {
        joint[LHY].targetRadian = l_q_cal(0);
        joint[LHR].targetRadian = l_q_cal(1);
        joint[LHP].targetRadian = l_q_cal(2);
        joint[LKN].targetRadian = l_q_cal(3);
        joint[LAP].targetRadian = l_q_cal(4);
        joint[LAR].targetRadian = l_q_cal(5);

        joint[RHY].targetRadian = r_q_cal(0);
        joint[RHR].targetRadian = r_q_cal(1);
        joint[RHP].targetRadian = r_q_cal(2);
        joint[RKN].targetRadian = r_q_cal(3);
        joint[RAP].targetRadian = r_q_cal(4);
        joint[RAR].targetRadian = r_q_cal(5);
    }


    //* Joint Controller
    ROSMsgPublish();
    jointController();
}

void gazebo::rok3_plugin::ROSMsgPublish(void)
{
    l_r_des_z.data[0] = l_r_des(2);
    L_R_DES_Z.publish(l_r_des_z);
}

void gazebo::rok3_plugin::jointController()
{
    /*
     * Joint Controller for each joint
     */

    // Update target torque by control
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = joint[j].Kp * (joint[j].targetRadian - joint[j].actualRadian)\
                              + joint[j].Kd * (joint[j].targetVelocity - joint[j].actualVelocity);
    }

    // Update target torque in gazebo simulation     
    L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
    L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
    L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
    L_Knee_joint->SetForce(0, joint[LKN].targetTorque);
    L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);
    L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);

    R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
    R_Knee_joint->SetForce(0, joint[RKN].targetTorque);
    R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);
    R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque);

    torso_joint->SetForce(0, joint[WST].targetTorque);
}

void gazebo::rok3_plugin::GetJoints()
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");
    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");
    torso_joint = this->model->GetJoint("torso_joint");

    //* FTsensor joint
    LS = this->model->GetJoint("LS");
    RS = this->model->GetJoint("RS");
}

void gazebo::rok3_plugin::GetjointData()
{
    /*
     * Get encoder and velocity data of each joint
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    joint[LHY].actualRadian = L_Hip_yaw_joint->GetAngle(0).Radian();
    joint[LHR].actualRadian = L_Hip_roll_joint->GetAngle(0).Radian();
    joint[LHP].actualRadian = L_Hip_pitch_joint->GetAngle(0).Radian();
    joint[LKN].actualRadian = L_Knee_joint->GetAngle(0).Radian();
    joint[LAP].actualRadian = L_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[LAR].actualRadian = L_Ankle_roll_joint->GetAngle(0).Radian();

    joint[RHY].actualRadian = R_Hip_yaw_joint->GetAngle(0).Radian();
    joint[RHR].actualRadian = R_Hip_roll_joint->GetAngle(0).Radian();
    joint[RHP].actualRadian = R_Hip_pitch_joint->GetAngle(0).Radian();
    joint[RKN].actualRadian = R_Knee_joint->GetAngle(0).Radian();
    joint[RAP].actualRadian = R_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[RAR].actualRadian = R_Ankle_roll_joint->GetAngle(0).Radian();

    joint[WST].actualRadian = torso_joint->GetAngle(0).Radian();

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
    }


    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    joint[WST].actualVelocity = torso_joint->GetVelocity(0);


    //    for (int j = 0; j < nDoF; j++) {
    //        cout << "joint[" << j <<"]="<<joint[j].actualDegree<< endl;
    //    }

}

void gazebo::rok3_plugin::initializeJoint()
{
    /*
     * Initialize joint variables for joint control
     */

    for (int j = 0; j < nDoF; j++) {
        joint[j].targetDegree = 0;
        joint[j].targetRadian = 0;
        joint[j].targetVelocity = 0;
        joint[j].targetTorque = 0;

        joint[j].actualDegree = 0;
        joint[j].actualRadian = 0;
        joint[j].actualVelocity = 0;
        joint[j].actualRPM = 0;
        joint[j].actualTorque = 0;
    }
}

void gazebo::rok3_plugin::SetJointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
    static double gain_p = 2.5;
    static double gain_d = 1.5;

    joint[LHY].Kp = 2000 * gain_p;
    joint[LHR].Kp = 9000 * gain_p;
    joint[LHP].Kp = 2000 * gain_p;
    joint[LKN].Kp = 5000 * gain_p;
    joint[LAP].Kp = 3000 * gain_p;
    joint[LAR].Kp = 3000 * gain_p;

    joint[RHY].Kp = joint[LHY].Kp;
    joint[RHR].Kp = joint[LHR].Kp;
    joint[RHP].Kp = joint[LHP].Kp;
    joint[RKN].Kp = joint[LKN].Kp;
    joint[RAP].Kp = joint[LAP].Kp;
    joint[RAR].Kp = joint[LAR].Kp;

    joint[WST].Kp = 2.;

    joint[LHY].Kd = 2. * gain_d;
    joint[LHR].Kd = 2. * gain_d;
    joint[LHP].Kd = 2. * gain_d;
    joint[LKN].Kd = 4. * gain_d;
    joint[LAP].Kd = 2. * gain_d;
    joint[LAR].Kd = 2. * gain_d;

    joint[RHY].Kd = joint[LHY].Kd;
    joint[RHR].Kd = joint[LHR].Kd;
    joint[RHP].Kd = joint[LHP].Kd;
    joint[RKN].Kd = joint[LKN].Kd;
    joint[RAP].Kd = joint[LAP].Kd;
    joint[RAR].Kd = joint[LAR].Kd;

    joint[WST].Kd = 2.;
}
