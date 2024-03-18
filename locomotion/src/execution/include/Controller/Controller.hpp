#pragma once


#include "kinodynamics.hpp"
#include <string>

#include "memory_types.hpp"
#include "utils.hpp"
#include "timer.hpp"
// #include "Iir.h"
#include <qpOASES.hpp>

using namespace qpOASES;

#define MU 0.5
#define GRAVITY 9.8

static const double NEGATIVE_NUMBER = -1000000.0;
static const double POSITIVE_NUMBER = 1000000.0;

class Controller {
public:
    Controller();

    Controller(const std::string&);

    ~Controller();

    vec12 GetDesiredContactForcePGD(const Eigen::MatrixXd& JabT, const vec6 &b);
    vec12 getDesiredContactForceqpOASES(const vec6 &b);

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

    void resize_qpOASES_vars();

    void resize_eigen_vars();
    
    void update_problem_size();

    void print_real_t(real_t *matrix, int nRows, int nCols);

    void copy_Eigen_to_real_t(real_t *target, Eigen::MatrixXd &source, int nRows, int nCols);

    void copy_real_t_to_Eigen(Eigen::VectorXd &target, real_t *source, int len);

    void print_QPData();

    Eigen::VectorXd clipVector(const Eigen::VectorXd &b, float F);

    void cleanFc(vec12& Fc);
private:
    std::string m_name;

    void updateEstimationData();
    void updatePlannerData();
    void updateJointCommand();

    void InitClass();
    
    // if starting from stance then set to false
    bool m_starting_from_sleep = false;
    
    vec12 F_prev;

    // float m_loop_rate = 1000;
    // Iir::Butterworth::LowPass<2> m_Fc_filter[12];
    // qpOASES related variables
    real_t *H_qpOASES;

    real_t *A_qpOASES;

    real_t *g_qpOASES;

    real_t *lbA_qpOASES;

    real_t *ubA_qpOASES;

    real_t *xOpt_qpOASES;

    real_t *xOpt_initialGuess;

    Eigen::MatrixXd H_eigen, A_eigen, g_eigen, lbA_eigen, ubA_eigen;

    Eigen::VectorXd xOpt_eigen;

    uint8_t real_allocated, num_vars_qp, num_constr_qp, c_st;

    double fz_min, fz_max;

    int_t qp_exit_flag;

    int_t nWSR_qpOASES;
    
    real_t cpu_time;
};