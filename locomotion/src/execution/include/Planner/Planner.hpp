#pragma once

#include "kinodynamics.hpp"

#include <string>

#include "Planner/Gait.hpp"
#include "Planner/FootSwingTrajectory.h"

#include "utils.hpp"
#include "memory_types.hpp"

class Planner {
public:
    Planner();
    Planner(std::string name);
    ~Planner();

    virtual Eigen::Vector3d GetDesiredFootPosition(const uint8_t&, const float&);
    virtual void SetFeetTarget();
    virtual void SetBaseTarget();
    virtual void SetTarget();

    void SetRobot(Quadruped& robot) {
        m_robot = robot;
        InitClass();
    }

    void SetGait(Gait& gait) {
        m_gait = gait;
        InitClass();
    }

    void setPlannerDataPtr(QuadrupedPlannerData* pd) {
        m_planner_data_ptr = pd;
    }
    void setEstimationDataPtr(QuadrupedEstimationData* est_ptr) {
        m_estimation_data_ptr = est_ptr;
    }

    void SetDesiredVelocity(const float& vx, const float& vy, const float& vyaw);

    virtual void Step();

    float m_t_curr = 0;
    Gait m_gait;

    bool m_check_safe_orientation = true;
    bool m_check_desired_footholds = true;

    void InitClass();

    void reset();
protected:
    Quadruped m_robot;
    Eigen::Vector4d getScheduledContactProbability(const float &t, Gait &gait);
    Eigen::Array4i m_cs_ref;
    int4 m_cs_act;
    Eigen::Vector4d p_cs_phi;
    float m_v_cmd_x = 0.0;
    float m_v_cmd_y = 0.0;
    float m_v_cmd_yaw = 0.0;
    float max_vel_x = 1.0;
    float max_vel_y = 1.0;
    float max_vel_yaw = 1.0;
    float m_loop_rate = 1000;
    float m_dt = 0.001;

    vec4 m_Pc_act;
    vec12 m_theta;

    vec19 m_joint_state_act;
    vec18 m_joint_vel_act;

    vec19 m_ee_state_ref;
    vec18 m_ee_vel_ref, m_ee_acc_ref;

private:
    Eigen::VectorXd GetHipPosition(const Eigen::VectorXd&);
    void updateExpectedStanceFlag();
    void UpdateTakeoffData();
    void adjustStanceLegs();

    double getTimeSinceStart();

    QuadrupedPlannerData* m_planner_data_ptr;
    QuadrupedEstimationData* m_estimation_data_ptr;
    void updatePlannerData();
    void updateEstimationData();

    // bool CheckSafeOrientation();
    // void CheckDesiredFootholds(Eigen::Vector3d&);

    std::string m_name;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_currentTimePoint;
    
    vec4 m_t_takeoff;
    vec12 m_p_takeoff_full;
    vec19 m_ee_state_init;
    int4 m_expected_stance;
    
    // reference foot trajectories
    FootSwingTrajectory<float> ref_foot_traj[4];
};