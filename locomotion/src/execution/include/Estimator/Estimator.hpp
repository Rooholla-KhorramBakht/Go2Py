#pragma once

#include "kinodynamics.hpp"
#include <string>
#include "memory_types.hpp"

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
    void setStepTime(const float& dt) {
        m_dt = dt;
    }

    void Step();
protected:
    Quadruped m_robot;

    // To write
    QuadrupedEstimationData est_data;
    // To read
    QuadrupedSensorData sensor_data;
    QuadrupedSensorData* m_sensor_data_ptr;
    QuadrupedEstimationData* m_estimation_data_ptr;
    QuadrupedPlannerData* m_planner_data_ptr;

    vec3 base_position;
    vec4 base_quat;
    vec3 base_vel;
    vec3 base_omega;

    vec4 m_pc_ref;
    float m_dt;
private:
    std::string m_name;

    void updateSensorData();
    void updateEstimationData();

    void InitClass();
};