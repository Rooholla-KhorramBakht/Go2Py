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

    m_fc_est = vec12::Zero();

    float sampling_freq = m_loop_rate;
    // std::cout << "Sampling freq: " << sampling_freq << "\n";
    float cutoff_freq_imu = 5;
    float cutoff_freq_encoder = 20;
    for (int i = 0; i < 12; ++i) {
        m_q_filter[i].setup(sampling_freq, cutoff_freq_encoder);
        m_qd_filter[i].setup(sampling_freq, cutoff_freq_encoder);
        m_tau_filter[i].setup(sampling_freq, cutoff_freq_encoder);
    }
    for (int i = 0; i < 3; ++i) {
        m_imu_a_filter[i].setup(sampling_freq, cutoff_freq_imu);
        m_imu_w_filter[i].setup(sampling_freq, cutoff_freq_imu);
    }
}

void Estimator::Step(const float& dt) {
    m_dt = dt;
    m_loop_rate = 1./m_dt;
    updateSensorData();
    ComputeEstimate();
    updateEstimationData();
}

void Estimator::reset() {
    est_data = QuadrupedEstimationData();
    m_fc_est = vec12::Zero();
    updateEstimationData();
}

void Estimator::updateSensorData() {
    sensor_data.copy(*m_sensor_data_ptr);

    for (int i = 0; i < 12; ++i) {
        sensor_data.q(i) = m_q_filter[i].filter(sensor_data.q(i));
        sensor_data.qd(i) = m_qd_filter[i].filter(sensor_data.qd(i));
        sensor_data.tau(i) = m_tau_filter[i].filter(sensor_data.tau(i));
    }
    for (int i = 0; i < 3; ++i) {
        sensor_data.a_B(i) = m_imu_a_filter[i].filter(sensor_data.a_B(i));
        sensor_data.w_B(i) = m_imu_w_filter[i].filter(sensor_data.w_B(i));
    }

    mat3x3 Rot = pinocchio::quat2rot(sensor_data.quat);

    sensor_data.a_W = Rot * sensor_data.a_B;
    sensor_data.w_W = Rot * sensor_data.w_B;

    m_pc_ref = m_planner_data_ptr -> pc_ref;
}
void Estimator::updateEstimationData() {
    m_estimation_data_ptr->copy(est_data);
    m_measurement_data_ptr -> estimated_contact_force = this -> m_fc_est;
}
// void Estimator::updateMeasurementData() {
    
// }