#pragma once

#include <thread>

// #include "SHM.hpp"
#include "QuadROSComm.hpp"
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
    float m_dt;
    int delay_ms = 10;

    void InitClass();

    // SHM m_plant_data;
    std::shared_ptr<QuadROSComm> m_plant_data_ptr;

    Quadruped m_robot;

    // Controller m_controller;
    DistributedIDC m_controller;
    QuadEstimator m_estimator;
    ReactivePlanner m_planner;

    QuadrupedSensorData m_sensor_data;
    QuadrupedCommandData m_cmd_data;
    QuadrupedEstimationData m_est_data;
    QuadrupedPlannerData m_planner_data;

    std::thread m_comm_thread;
};