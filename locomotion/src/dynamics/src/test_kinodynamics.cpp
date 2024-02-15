#include "kinodynamics.hpp"


#include <string>
#include <iostream>

class Dummy {
    Quadruped robot;
};

int main(int argc, char** argv) {

    std::string robot_name = "go2";
    std::string package_name = "go2_description";
    // std::string urdf_filepath = ros::package::getPath(package_name) + "/urdf/" + robot_name + "_description.urdf";

    std::string urdf_filepath = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/b1_description/urdf/b1_description.urdf";

    std::cout << "URDF filepath: " << urdf_filepath << "\n";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filepath, pinocchio::JointModelFreeFlyer(), model);
    // model.gravity = Eigen::Vector3d(0, 0, 0);
    pinocchio::Data data(model);

    /********************* Probe the model and data objects to know the data in them *********************/

    std::cout << "Name: " << model.name << "\n";
    int dof = model.nq;
    std::cout << "dof = " << dof << std::endl;

    for (int i = 0; i < model.names.size(); ++i) {
        std::cout << "ID: " << i << "\t Joint: " << model.names[i] << std::endl;
    }
    for (int i = 0; i < model.frames.size(); ++i) {
        std::cout << "ID: " << i << "\t Frame: " << model.frames[i].name << std::endl;
    }

    /*******************************************************************************************/

    // Sample a random configuration
    Eigen::VectorXd lower_limit = -10 * Eigen::VectorXd::Ones(19);
    Eigen::VectorXd upper_limit = 10 * Eigen::VectorXd::Ones(19);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model, lower_limit, upper_limit);
    std::cout << "q: " << q.transpose() << std::endl;
    std::cout << "Size: " << q.size() << std::endl;

    Eigen::VectorXd q_test = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd q_translate = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd q_stretch = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd q_stand_pitch = Eigen::VectorXd::Zero(19);
    q_stretch(2) = 0.42;
    q_stretch(6) = 1;

    q_test << 0.0, 0.0, 0.34, 0.0, 0.0, 0.0, 1.0, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3;
    q_translate << 5.0, 3.0, 0.34, 0.0, 0.0, 0.0, 1.0, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3;
    q_stand_pitch << 0.0, 0.0, 0.34, 0, 0.247404, 0, 0.9689124, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3;
    /******      Forward Kinematics routine     *******/

    // First calls the forwardKinematics on the model, then computes the placement of each frame.
    pinocchio::framesForwardKinematics(model, data, q_translate);

    // extract position of the foot link frame and return
    // return data.oMf;
    Eigen::VectorXd feet_pos = Eigen::VectorXd::Zero(12);

    int IDX_FOOT_FL = model.getFrameId("FL_foot");
    int IDX_FOOT_FR = model.getFrameId("FR_foot");
    int IDX_FOOT_RL = model.getFrameId("RL_foot");
    int IDX_FOOT_RR = model.getFrameId("RR_foot");
    int IDX_BASE = model.getFrameId("base");

    // std::cout << "FR position: " << data.oMf[IDX_FOOT_FR].translation() << "\n";
    
    feet_pos.block<3, 1>(0, 0) = data.oMf[IDX_FOOT_FL].translation();
    feet_pos.block<3, 1>(3, 0) = data.oMf[IDX_FOOT_FR].translation();
    feet_pos.block<3, 1>(6, 0) = data.oMf[IDX_FOOT_RL].translation();
    feet_pos.block<3, 1>(9, 0) = data.oMf[IDX_FOOT_RR].translation();

    std::cout << "feet pos: " << feet_pos.transpose() << std::endl;
    std::cout << "feet rot FL : " << data.oMf[IDX_FOOT_FL].rotation() << std::endl;

    Quadruped robot(urdf_filepath.c_str());

    std::cout << "Class FK: " << robot.forwardKinematics(q_stretch).transpose() << std::endl;

    /*************** Get Jacobians for feet frame ***************/
    q = q_test;
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::framesForwardKinematics(model, data, q);
    Eigen::Matrix<double, 6, 18> tmp_jac;
    // pinocchio::getFrameJacobian(model, data, IDX_FOOT_FR, pinocchio::WORLD, tmp_jac);

    pinocchio::getFrameJacobian(model, data, IDX_BASE, pinocchio::LOCAL, tmp_jac);
    pinocchio::getFrameJacobian(model, data, IDX_BASE, pinocchio::LOCAL_WORLD_ALIGNED, tmp_jac);
    pinocchio::getFrameJacobian(model, data, IDX_FOOT_FL, pinocchio::LOCAL, tmp_jac);
    pinocchio::getFrameJacobian(model, data, IDX_FOOT_FL, pinocchio::LOCAL_WORLD_ALIGNED, tmp_jac);

    
    // std::cout << "Base Jacobian LOCAL: \n" << tmp_jac << "\n";
    // std::cout << "Base Jacobian WORLD: \n" << tmp_jac << "\n";
    // std::cout << "Foot FL Jacobian LOCAL: \n" << tmp_jac << "\n";
    // std::cout << "Foot FL Jacobian WORLD: \n" << tmp_jac << "\n";

    // std::cout << "Feet Jacobian: \n" << robot.feetJacobian(q) << std::endl;

    // std::cout << "Mass Matrix: \n" << robot.massMatrix(q) << std::endl;

    /***************************** Inverse Kinematics computation ***************************/

    std::cout << "Neutral: " << pinocchio::neutral(model).transpose() << "\n";
    
    Eigen::VectorXd q_curr = pinocchio::neutral(model).transpose();
    q_curr(0) = 0.05;
    q_curr.block<4,1>(3, 0) = vec4(0.0967298, 0.246168, 0.0246992, 0.9640719);

    pinocchio::framesForwardKinematics(model, data, q_curr);
    std::cout << "Current base orientation: \n" << data.oMf[IDX_BASE].rotation() << std::endl;
    std::cout << "Custom function: \n" << pinocchio::quat2rot(q_curr.block<4,1>(3, 0)) << std::endl;

    Eigen::VectorXd q_target = q_test;

    Eigen::VectorXd feet_pos_target = robot.forwardKinematics(q_target);

    vec19 ee_state_target = vec19::Zero();

    ee_state_target.block<7,1>(0, 0) = q_target.block<7,1>(0, 0);
    ee_state_target.block<12,1>(7, 0) = feet_pos_target;

    std::cout << "ee target: " << ee_state_target.transpose() <<  "\n";

    std::cout << "Joint angles: " << robot.inverseKinematics(ee_state_target).transpose() << "\n";

    vec18 qv = vec18::Zero();

    std::cout << "Sleep states: " << robot.getSleepStates().transpose() << std::endl;
    std::cout << "Stance states: " << robot.getStanceStates().transpose() << std::endl;

    Quadruped robot2;
    robot2 = robot;
    std::cout << "robot2 default filepath: " << robot2.getURDFFilepath() << std::endl;
    // robot2 = *robot;

    std::cout << "Robot 2 Sleep: " << robot2.getSleepStates().transpose() << std::endl;
    
    std::cout << "Mass Matrix: \n" << robot.massMatrix(q_test) << std::endl;
    std::cout << "nonLinearEffects: " << robot.nonLinearEffects(q_test, qv).transpose() << std::endl;
    std::cout << "grav vector: " << robot.gravitationalTerms(q_test).transpose() << std::endl;
    std::cout << "coriolis vector: " << robot.coriolisVector(q_test, qv).transpose() << std::endl;
    std::cout << "Coriolis Matrix: \n" << robot.coriolisMatrix(q_test, qv) << std::endl;
}