#pragma once

#include <thread>

// #include "CommunicationManager.hpp"
#include "Controller/DistributedIDC.hpp"
// #include "Controller/RobustDIDC.hpp"
#include "Estimator/QuadEstimator.hpp"
#include "Planner/ReactivePlanner.hpp"

#if defined(USE_ROS_COMM)
#include "QuadROSComm.hpp"
#elif defined(USE_DDS_COMM)
#include "QuadDDSComm.hpp"
#else
#include "SHM.hpp"
#endif

class Executor {
public:
    Executor();
    Executor(const std::string& name);
    ~Executor();

    void setUpdateRate(const float& rate);

    void step();

    void run();
private:
    std::string m_name;
    float m_rate = 1000;
    float m_dt = 0.001;
    double t_curr = 0;
    float t_last = 0;
    int delay_ms = 1;
    int exec_time_ms = 0;

    bool m_gait_switch = true;

    void initClass();

    double updateTimer();

    // void switchGait(const Gait& gait);

    // SHM m_plant_data;
#if defined(USE_ROS_COMM)
    std::shared_ptr<QuadROSComm> m_plant_data_ptr;
#elif defined(USE_DDS_COMM)
    std::shared_ptr<QuadDDSComm> m_plant_data_ptr;
#else
    std::shared_ptr<SHM> m_plant_data_ptr;
#endif

    Quadruped m_robot;

    // Controller m_controller;
    DistributedIDC m_controller;
    // RobustDIDC m_controller;
    QuadEstimator m_estimator;
    ReactivePlanner m_planner;
    // Planner m_planner;

    Gait stance;
    Gait trot;
    Gait walk;

    QuadrupedSensorData m_sensor_data;
    QuadrupedCommandData m_cmd_data;
    QuadrupedEstimationData m_est_data;
    QuadrupedPlannerData m_planner_data;
    QuadrupedMeasurementData m_measurement_data;
    QuadrupedJoystickData m_joystick_data;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_currentTimePoint;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastTimePoint;
    std::thread m_comm_thread;

    bool use_plant_time = false;
};