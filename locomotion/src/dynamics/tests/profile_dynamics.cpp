#include "kinodynamics.hpp"
#include "timer.hpp"


#include <string>
#include <iostream>

int main(int argc, char** argv) {

    std::string robot_name = "go2";
    std::string package_name = "go2_description";
    // std::string urdf_filepath = ros::package::getPath(package_name) + "/urdf/" + robot_name + "_description.urdf";

    // std::string urdf_filepath = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/" + robot_name + "_description/urdf/" + robot_name + "_description.urdf";
    std::string urdf_filepath = "/root/dev/xMo/src/robots/" + robot_name + "_description/urdf/" + robot_name + "_description.urdf";


    Quadruped robot(urdf_filepath.c_str());

    Eigen::VectorXd q_test = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd q_translate = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd q_stretch = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd q_stand_pitch = Eigen::VectorXd::Zero(19);
    q_stretch(2) = 0.42;
    q_stretch(6) = 1;

    q_test << 0.0, 0.0, 0.34, 0.0, 0.0, 0.0, 1.0, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3;
    q_translate << 5.0, 3.0, 0.34, 0.0, 0.0, 0.0, 1.0, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3;
    q_stand_pitch << 0.0, 0.0, 0.34, 0, 0.247404, 0, 0.9689124, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3;
    vec18 qv = vec18::Zero();


    {
        // Time FK
        Timer timer("Pinocchio FK");
        robot.forwardKinematics(q_stretch);
    }
    Eigen::VectorXd q_target = q_test;
    Eigen::VectorXd feet_pos_target = robot.forwardKinematics(q_target);

    vec19 ee_state_target = vec19::Zero();

    ee_state_target.block<7,1>(0, 0) = q_target.block<7,1>(0, 0);
    ee_state_target.block<12,1>(7, 0) = feet_pos_target;

    // std::cout << "ee target: " << ee_state_target.transpose() <<  "\n";

    // std::cout << "Joint angles: " << robot.inverseKinematics(ee_state_target).transpose() << "\n";

    {
        // Time IK
        Timer timer("Pinocchio IK");
        robot.inverseKinematics(ee_state_target);
    }

    {
        // Time Feet Jacobian
        Timer timer("Pinocchio feetJacobian");
        robot.feetJacobian(q_test);
    }
    {
        // Time Feet Jacobian Dot
        Timer time("Pinocchio feetJacobianDot");
        robot.feetJacobianDot(q_test, qv);
    }
    {
        Timer timer("Pinocchio Mass Matrix");
        robot.massMatrix(q_test);
    }
    {
        Timer timer("Pinocchio non linear terms");
        robot.nonLinearEffects(q_test, qv);
    }
    {
        Timer timer("Pinocchio coriolis vector");
        robot.coriolisVector(q_test, qv);
    }
    {
        Timer timer("Pinocchio Grav Vector");
        robot.gravitationalTerms(q_test);
    }

    return 0;
}