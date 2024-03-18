#pragma once

#include "memory_types.hpp"
#include <string>
#include <iostream>

enum DATA_ACCESS_MODE {
    PLANT,
    EXECUTOR
};

// Single communication manager for quadruped
class CommunicationManager {
public:
    CommunicationManager();
    CommunicationManager(const DATA_ACCESS_MODE& mode);
    CommunicationManager(const std::string&, const DATA_ACCESS_MODE& mode);
    ~CommunicationManager() {}

    void setSensorDataPtr(QuadrupedSensorData* sensor_data_ptr) {
        m_sensor_data_ptr = sensor_data_ptr;
    }
    void setCommandDataPtr(QuadrupedCommandData* command_data_ptr) {
        m_joint_command_data_ptr = command_data_ptr;
    }
    void setMeasurementDataPtr(QuadrupedMeasurementData* measurement_data_ptr) {
        m_measurement_data_ptr = measurement_data_ptr;
    }
    void setEstimationDataPtr(QuadrupedEstimationData* estimation_data_ptr) {
        m_estimation_data_ptr = estimation_data_ptr;
    }
    void setJoystickDataPtr(QuadrupedJoystickData* joystick_data_ptr) {
        m_joystick_data_ptr = joystick_data_ptr;
    }
    void setPlantTimePtr(double* time_ptr) {
        m_plant_time_ptr = time_ptr;
    }

    void writeSensorData(const QuadrupedSensorData& sensor_data);
    void writeCommandData(const QuadrupedCommandData& cmd_data);
    void writeMeasurementData(const QuadrupedMeasurementData& measure_data);
    void writeEstimationData(const QuadrupedEstimationData& est_data);
    void writeJoystickData(const QuadrupedJoystickData& joy_data);
    void writePlantTime(const double& time);
    
    void getSensorData(QuadrupedSensorData& sensor_data);
    void getCommandData(QuadrupedCommandData& cmd_data);
    void getMeasurememtData(QuadrupedMeasurementData& measure_data);
    void getEstimationData(QuadrupedEstimationData& est_data);
    void getJoystickData(QuadrupedJoystickData& joy_data);
    void getPlantTime(double& time);

    void setAccessMode(const DATA_ACCESS_MODE& mode) {
        m_mode = mode;
    }

protected:
    QuadrupedSensorData* m_sensor_data_ptr = NULL;
    QuadrupedCommandData* m_joint_command_data_ptr = NULL;
    QuadrupedEstimationData* m_estimation_data_ptr = NULL;
    QuadrupedMeasurementData* m_measurement_data_ptr = NULL;
    QuadrupedJoystickData* m_joystick_data_ptr = NULL;
    double* m_plant_time_ptr = NULL;

    int m_mode = DATA_ACCESS_MODE::PLANT;
private:
    std::string m_name;
};