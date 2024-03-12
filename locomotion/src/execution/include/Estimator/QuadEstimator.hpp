#pragma once

#include "Estimator.hpp"

class QuadEstimator : public Estimator {
public:
    QuadEstimator();
    QuadEstimator(std::string name);
    ~QuadEstimator() {}

    void ComputeEstimate() override;
    void InitClass();
private:
    void UpdatePoseEstimate();
    void UpdateContactEstimate();
    inline float get_pc_pz(const float& pz, const float& mu_pz, const float & sigma_pz);
    inline float get_pc_fz(const float& fz, const float& mu_fz, const float & sigma_fz);
    vec4 get_pc_pz(const vec4& pz);
    vec4 get_pc_fz(const vec4& fz);

    std::string m_name;

    vec12 m_forces_estimated;
    
    mat4x4 m_sigma_pz;
    mat4x4 m_sigma_fz;
    mat8x8 m_sigma_measurement;
    mat4x4 m_sigma_process;

    float process_noise_pimu;
    float process_noise_vimu;
    float process_noise_pfoot;
    float sensor_noise_pimu_rel_foot;
    float sensor_noise_vimu_rel_foot;
    float sensor_noise_zfoot;

    // Estimator Vars
    Eigen::Matrix<double, 18, 1> _xhat;
    Eigen::Matrix<double, 12, 1> _ps;
    Eigen::Matrix<double, 12, 1> _vs;
    Eigen::Matrix<double, 18, 18> _A;
    Eigen::Matrix<double, 18, 18> _Q0;
    Eigen::Matrix<double, 18, 18> _P;
    Eigen::Matrix<double, 28, 28> _R0;
    Eigen::Matrix<double, 18, 3> _B;
    Eigen::Matrix<double, 28, 18> _C;
    Eigen::Matrix<double, 18, 18> Q;
    Eigen::Matrix<double, 28, 28> R;

    void InitEstimatorVars();
    void InitTimedEstimatorVars();

    vec18 m_jv_prev;
    // Iir::Butterworth::LowPass<2> m_ja_filter[18];
};