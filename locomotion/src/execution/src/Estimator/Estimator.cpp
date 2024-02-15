#include "Estimator/Estimator.hpp"

Estimator::Estimator() : m_name("svan") {
    InitClass();
}

Estimator::Estimator(const std::string& name) : m_name(name) {
    InitClass();
}

void Estimator::InitClass() {
    base_position = vec3::Zero();
    base_quat = vec4::Zero();
    base_vel = vec3::Zero();
    base_omega = vec3::Zero();

    m_pc_ref = Eigen::Vector4d::Ones();
}

void Estimator::Step() {
    updateSensorData();
    ComputeEstimate();
    updateEstimationData();
}

void Estimator::updateSensorData() {
    sensor_data.copy(*m_sensor_data_ptr);
    m_pc_ref = m_planner_data_ptr -> pc_ref;
}
void Estimator::updateEstimationData() {
    m_estimation_data_ptr->copy(est_data);
}