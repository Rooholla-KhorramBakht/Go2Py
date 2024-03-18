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

    vec3 getRaibertHeuristic(const uint8_t& leg_id, const float& t_stance);
    virtual Eigen::Vector3d getDesiredFootPosition(const uint8_t&, const float&);
    virtual void setFeetTarget();
    virtual void setBaseTarget();
    virtual void setTarget();

    void setRobot(Quadruped& robot) {
        m_robot = robot;
        initClass();
    }

    void setGait(Gait& gait) {
        m_gait = gait;
        // m_gait.updateInitTime(m_t_curr);
    }

    void setPlannerDataPtr(QuadrupedPlannerData* pd) {
        m_planner_data_ptr = pd;
    }
    void setEstimationDataPtr(QuadrupedEstimationData* est_ptr) {
        m_estimation_data_ptr = est_ptr;
    }

    void setDesiredVelocity(const float& vx, const float& vy, const float& vyaw);
    bool setTargetBasePosition(const vec3& target_pos, const float& target_yaw);
    bool sleepToStance();
    bool stanceToSleep();
    
    void setStance();

    virtual void step(const float& dt, const float& t_curr);

    float m_t_curr = 0;
    Gait m_gait;

    bool m_check_safe_orientation = true;
    bool m_check_desired_footholds = true;

    void initClass();

    void reset();

    void startFromSleep(const bool& sleep_start);
protected:
    Quadruped m_robot;
    Eigen::Vector4d getScheduledContactProbability(const float &t, Gait &gait);
    Eigen::Array4i m_cs_ref;
    int4 m_cs_act;
    Eigen::Vector4d p_cs_phi;
    double m_v_cmd_x = 0.0;
    double m_v_cmd_y = 0.0;
    double m_v_cmd_z = 0.0;
    double m_v_cmd_yaw = 0.0;
    double m_a_max_x = 0.2;
    double m_a_max_y = 0.2;
    double m_a_max_z = 0.2;
    double m_a_max_yaw = 0.2;
    double max_vel_x = 1.0;
    double max_vel_y = 1.0;
    double max_vel_z = 1.0;
    double max_vel_yaw = 1.0;
    double m_loop_rate = 1000;
    double m_dt = 0.001;

    double m_yaw = 0;

    // vec12 p_step_;

    vec3 m_target_base_position;
    double m_target_yaw;

    vec4 m_Pc_act;
    vec12 m_theta;

    vec19 m_joint_state_act;
    vec18 m_joint_vel_act;

    vec19 m_ee_state_ref;
    vec18 m_ee_vel_ref, m_ee_acc_ref;

    vec12 m_p_takeoff_full;

private:
    Eigen::VectorXd getHipPosition(const Eigen::VectorXd&);
    void updateExpectedStanceFlag();
    void updateTakeoffData();
    void adjustStanceLegs();

    double getTimeSinceStart();

    QuadrupedPlannerData* m_planner_data_ptr;
    QuadrupedEstimationData* m_estimation_data_ptr;
    void updatePlannerData();
    void updateEstimationData();

    // bool CheckSafeOrientation();
    // void CheckDesiredFootholds(Eigen::Vector3d&);

    bool m_sleep_start = false;

    std::string m_name;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_currentTimePoint;
    
    vec4 m_t_takeoff;
    vec19 m_ee_state_init;
    int4 m_expected_stance;
    
    // reference foot trajectories
    FootSwingTrajectory<float> ref_foot_traj[4];
};