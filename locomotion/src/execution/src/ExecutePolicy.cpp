#include "ExecutorPolicy.hpp"
#include <chrono>
#include <thread>
#include <filesystem>

ExecutorPolicy::ExecutorPolicy() : m_name("svan") {
    InitClass();
}
ExecutorPolicy::ExecutorPolicy(const std::string& name) : m_name(name) {
    InitClass();
}

ExecutorPolicy::~ExecutorPolicy() {
    if (m_comm_thread.joinable()) {
        m_comm_thread.join();
    }
}

void ExecutorPolicy::setUpdateRate(const float& rate) {
    m_rate = rate;
    m_dt = 1./m_rate;
}

void ExecutorPolicy::InitClass() {
    m_rate = 1000;
    m_dt = 1./m_rate;

    m_plant_data_ptr = std::make_shared<QuadROSComm>(m_name);
    // m_plant_data->setAccessMode(DATA_ACCESS_MODE::EXECUTOR_TO_PLANT);
    m_plant_data_ptr->setSensorDataPtr(&m_sensor_data);
    m_plant_data_ptr->setCommandDataPtr(&m_cmd_data);
}

void ExecutorPolicy::step() {
    Timer timer("ExecutorPolicy Step");
    // m_plant_data.getSensorData(m_sensor_data);
    m_plant_data_ptr->getSensorData(m_sensor_data);
    
    update_observation_buffer();

    update_policy_action();

    update_command_from_action();

    m_plant_data_ptr->writeCommandData(m_cmd_data);

    // Sleep for specified time to maintain update rate
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

void ExecutorPolicy::run() {
    delay_ms = int(m_dt * double(1e3));

    m_comm_thread = std::thread(&QuadROSComm::Run, m_plant_data_ptr);
    m_comm_thread.detach();

    while (true) {
        step();
    }
}