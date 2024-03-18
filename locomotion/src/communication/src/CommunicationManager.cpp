#include "CommunicationManager.hpp"

CommunicationManager::CommunicationManager() : m_mode(DATA_ACCESS_MODE::PLANT) {
}
CommunicationManager::CommunicationManager(const DATA_ACCESS_MODE& mode) : m_name("svan"), m_mode(mode) {
}

CommunicationManager::CommunicationManager(const std::string& name, const DATA_ACCESS_MODE& mode) : m_name(name), m_mode(mode) {
}

void CommunicationManager::writeSensorData(const QuadrupedSensorData& sensor_data) {
    if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        std::cout << "Access mode set to only read sensor data. Cannot Write. Operation failed.\n";
        return;
    }
    if (m_sensor_data_ptr == NULL) {
        std::cout << "Could not get the address to write sensor data...\n";
        return;
    }
    m_sensor_data_ptr->copy(sensor_data);
}

void CommunicationManager::writeMeasurementData(const QuadrupedMeasurementData& measure_data) {
    if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        std::cout << "Access mode set to only read measurement data. Cannot Write. Operation failed.\n";
        return;
    }
    if (m_measurement_data_ptr == NULL) {
        std::cout << "Could not get the address to write measurement data...\n";
        return;
    }
    m_measurement_data_ptr->copy(measure_data);
}

void CommunicationManager::writeCommandData(const QuadrupedCommandData& cmd_data) {
    if (m_mode == DATA_ACCESS_MODE::PLANT) {
        std::cout << "Access mode set to only read command data. Cannot Write. Operation failed.\n";
        return;
    }
    if (m_joint_command_data_ptr == NULL) {
        std::cout << "Could not get the address to write command data...\n";
        return;
    }
    m_joint_command_data_ptr->copy(cmd_data);
}

void CommunicationManager::writeEstimationData(const QuadrupedEstimationData& est_data) {
    if (m_mode == DATA_ACCESS_MODE::PLANT) {
        std::cout << "Access mode set to only read command data. Cannot Write. Operation failed.\n";
        return;
    }
    if (m_estimation_data_ptr == NULL) {
        std::cout << "Could not get the address to write estimation data...\n";
        return;
    }
    m_estimation_data_ptr->copy(est_data);
}

void CommunicationManager::writeJoystickData(const QuadrupedJoystickData& joy_data) {
    // if (m_mode == DATA_ACCESS_MODE::PLANT) {
    //     std::cout << "Access mode set to only read command data. Cannot Write. Operation failed.\n";
    //     return;
    // }
    if (m_joystick_data_ptr == NULL) {
        std::cout << "Could not get the address to write estimation data...\n";
        return;
    }
    m_joystick_data_ptr->copy(joy_data);
}

void CommunicationManager::getSensorData(QuadrupedSensorData& sensor_data) {
    // if (m_mode == DATA_ACCESS_MODE::PLANT_TO_EXECUTOR) {
    //     std::cout << "Access mode set to only write sensor data. Cannot read. Operation failed.\n";
    //     return;
    // }
    if (m_sensor_data_ptr == NULL) {
        std::cout << "Memory not initialized. Cannot read.\n";
    }
    sensor_data.copy(*m_sensor_data_ptr);
}

void CommunicationManager::getMeasurememtData(QuadrupedMeasurementData& measure_data) {
    // if (m_mode == DATA_ACCESS_MODE::PLANT_TO_EXECUTOR) {
    //     std::cout << "Access mode set to only write measurement data. Cannot read. Operation failed.\n";
    //     return;
    // }
    if (m_measurement_data_ptr == NULL) {
        std::cout << "Memory not initialized. Cannot read.\n";
    }
    measure_data.copy(*m_measurement_data_ptr);
}

void CommunicationManager::getCommandData(QuadrupedCommandData& cmd_data) {
    // if (m_mode == DATA_ACCESS_MODE::EXECUTOR_TO_PLANT) {
    //     std::cout << "Access mode set to only write command data. Cannot read. Operation failed.\n";
    //     return;
    // }
    if (m_joint_command_data_ptr == NULL) {
        std::cout << "Memory not initialized. Cannot read.\n";
    }
    cmd_data.copy(*m_joint_command_data_ptr);
}
void CommunicationManager::getEstimationData(QuadrupedEstimationData& est_data) {
    // if (m_mode == DATA_ACCESS_MODE::EXECUTOR_TO_PLANT) {
    //     std::cout << "Access mode set to only write command data. Cannot read. Operation failed.\n";
    //     return;
    // }
    if (m_estimation_data_ptr == NULL) {
        std::cout << "Memory not initialized. Cannot read.\n";
    }
    est_data.copy(*m_estimation_data_ptr);
}
void CommunicationManager::getJoystickData(QuadrupedJoystickData& joy_data) {
    // if (m_mode == DATA_ACCESS_MODE::EXECUTOR_TO_PLANT) {
    //     std::cout << "Access mode set to only write command data. Cannot read. Operation failed.\n";
    //     return;
    // }
    if (m_joystick_data_ptr == NULL) {
        std::cout << "Memory not initialized. Cannot read.\n";
    }
    joy_data.copy(*m_joystick_data_ptr);
}

void CommunicationManager::getPlantTime(double& time) {
    if (m_plant_time_ptr != NULL) {
        time = *m_plant_time_ptr;
    } else {
        std::cout << "Time pointer set to NULL.\n";
    }
}

void CommunicationManager::writePlantTime(const double& time) {
    if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        std::cout << "Access mode executor. Cannot set time.\n";
        return;
    }
    if (m_plant_time_ptr != NULL) {
        *m_plant_time_ptr = time;
    } else {
        std::cout << "Time pointer set to NULL.\n";
    }
}