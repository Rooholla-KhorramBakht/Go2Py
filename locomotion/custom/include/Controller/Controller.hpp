#pragma once


#include "kinodynamics.hpp"
#include <string>

#include "memory_types.hpp"
#include "utils.hpp"
#include "timer.hpp"

class Controller {
public:
    Controller();

    Controller(const std::string&);

    ~Controller();

    vec12 GetDesiredContactForcePGD(const vec6 &b);

    virtual vec12 CalculateFeedForwardTorque() {
        return vec12::Zero();
    }

    // void CheckRobotInitialState();

    void Step();

    void SetRobot(Quadruped &robot) {
        m_robot = robot;
        InitClass();
    }

    void setEstimationDataPtr(QuadrupedEstimationData* est_data_ptr) {
        m_estimation_data_ptr = est_data_ptr;
    }
    void setPlannerDataPtr(QuadrupedPlannerData* planner_data_ptr) {
        m_planner_data_ptr = planner_data_ptr;
    }
    void setJointCommandDataPtr(QuadrupedCommandData* cmd_data_ptr) {
        m_joint_command_ptr = cmd_data_ptr;
    }

    void update_contact_flag_for_controller();

    void startFromSleep(const bool& sleep_start) {
        m_starting_from_sleep = sleep_start;
        if (sleep_start) {
            std::cout << "Controller starting from sleep...\n";
        } else {
            std::cout << "Controller starting from stance...\n";
        }
        InitClass();
    }

    Quadruped m_robot;

    vec19 m_joint_state_ref, m_ee_state_init, m_ee_state_default_stance, m_joint_state_init;
    vec19 m_ee_state_ref, m_joint_state_act;
    vec18 m_ee_vel_ref, m_ee_acc_ref, m_joint_vel_act, m_joint_acc_act;
    vec18 m_joint_vel_ref;
    vec12 m_ff_torque;
    int4 m_contact_flag_for_controller;
    int m_gait_id;

    int4 m_cs, m_expected_stance_flag;
    int m_num_contact;

    // Total uncertainities
    vec18 m_eta;
    // To calculate the publish the state error if required
    vec19 m_state_error;
protected:
    // To write
    QuadrupedCommandData* m_joint_command_ptr;
    // To read
    QuadrupedEstimationData* m_estimation_data_ptr;
    QuadrupedPlannerData* m_planner_data_ptr;
private:
    std::string m_name;

    void updateEstimationData();
    void updatePlannerData();
    void updateJointCommand();

    void InitClass();
    
    // if starting from stance then set to false
    bool m_starting_from_sleep = false;
    
    vec12 F_prev;
};