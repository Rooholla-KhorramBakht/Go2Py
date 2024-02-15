#pragma once

#include "memory_types.hpp"
#include <string>
#include <iostream>

enum DATA_ACCESS_MODE {
    PLANT_TO_EXECUTOR,
    EXECUTOR_TO_PLANT
};

// Single communication manager for quadruped
class CommunicationManager {
public:
    CommunicationManager(const DATA_ACCESS_MODE& mode);
    CommunicationManager(const std::string&, const DATA_ACCESS_MODE& mode);
    ~CommunicationManager() {}

    void writeSensorData(const QuadrupedSensorData& sensor_data);
    void writeCommandData(const QuadrupedCommandData& cmd_data);

    void setSensorDataPtr(QuadrupedSensorData* sensor_data_ptr) {
        m_sensor_data_ptr = sensor_data_ptr;
    }
    void setCommandDataPtr(QuadrupedCommandData* command_data_ptr) {
        m_joint_command_data_ptr = command_data_ptr;
    }

    void getSensorData(QuadrupedSensorData& sensor_data);
    void getCommandData(QuadrupedCommandData& cmd_data);

    void setAccessMode(const DATA_ACCESS_MODE& mode) {
        m_mode = mode;
    }

protected:
    QuadrupedSensorData* m_sensor_data_ptr = NULL;
    QuadrupedCommandData* m_joint_command_data_ptr = NULL;

    int m_mode = DATA_ACCESS_MODE::PLANT_TO_EXECUTOR;
private:
    std::string m_name;
    // QuadrupedEstimatoinData m_estimation_data;
    // QuadrupedPlannerData m_planner_data;
};