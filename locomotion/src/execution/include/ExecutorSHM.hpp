#pragma once

#include <thread>

#include "SHM.hpp"
// #include "QuadROSComm.hpp"
#include "Controller/DistributedIDC.hpp"
#include "Estimator/QuadEstimator.hpp"
#include "Planner/ReactivePlanner.hpp"

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
    float m_rate;
    float m_dt = 0;
    float t_curr = 0;
    float t_last = 0;
    int delay_ms = 1;
    int exec_time_ms = 0;

    void InitClass();
    
    double updateTimer();

    std::shared_ptr<SHM> m_plant_data_ptr;
    // std::shared_ptr<QuadROSComm> m_plant_data_ptr;

    Quadruped m_robot;

    Controller m_controller;
    // DistributedIDC m_controller;
    QuadEstimator m_estimator;
    ReactivePlanner m_planner;

    QuadrupedSensorData m_sensor_data;
    QuadrupedCommandData m_cmd_data;
    QuadrupedEstimationData m_est_data;
    QuadrupedPlannerData m_planner_data;
    QuadrupedMeasurementData m_measurement_data;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_currentTimePoint;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastTimePoint;
    // std::thread m_comm_thread;
};