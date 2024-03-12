#pragma once

#include "kinodynamics.hpp"
#include <string>
#include "memory_types.hpp"
#include "Iir.h"

class Estimator {
public:
    Estimator();
    Estimator(const std::string& name);
    ~Estimator() {}

    void SetRobot(Quadruped& robot) {
        m_robot = robot;
        InitClass();
    }

    virtual void ComputeEstimate() {}

    void setSensorDataPtr(QuadrupedSensorData* sd) {
        m_sensor_data_ptr = sd;
    }
    void setEstimationDataPtr(QuadrupedEstimationData* esd) {
        m_estimation_data_ptr = esd;
    }
    void setPlannerDataPtr(QuadrupedPlannerData* pd) {
        m_planner_data_ptr = pd;
    }
    void setMeasurementDataPtr(QuadrupedMeasurementData* md) {
        m_measurement_data_ptr = md;
    }
    void setLoopRate(const float& rate) {
        m_loop_rate = rate;
        m_dt = 1./rate;
        InitClass();
    }

    void Step(const float& dt);

    void reset();
protected:
    Quadruped m_robot;

    // To write
    QuadrupedEstimationData est_data;
    // To read
    QuadrupedSensorData sensor_data;
    QuadrupedSensorData* m_sensor_data_ptr;
    QuadrupedEstimationData* m_estimation_data_ptr;
    QuadrupedPlannerData* m_planner_data_ptr;
    QuadrupedMeasurementData* m_measurement_data_ptr;

    vec3 base_position;
    vec4 base_quat;
    vec3 base_vel;
    vec3 base_omega;

    vec4 m_pc_ref;

    vec12 m_fc_est;
    float m_dt;
    float m_loop_rate = 1000;
private:
    std::string m_name;

    void updateSensorData();
    void updateEstimationData();
    // void updateMeasurementData();

    void InitClass();

    Iir::Butterworth::LowPass<2> m_q_filter[12];
    Iir::Butterworth::LowPass<2> m_qd_filter[12];
    Iir::Butterworth::LowPass<2> m_tau_filter[12];
    Iir::Butterworth::LowPass<2> m_imu_a_filter[3];
    Iir::Butterworth::LowPass<2> m_imu_w_filter[3];
};