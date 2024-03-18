#pragma once

#include <string>
#include <eigen3/Eigen/Dense>

class Gait {
public:
    enum GAIT_TYPE {
        STANCE,
        TROT,
    };
    
    Gait();

    Gait(GAIT_TYPE);

    Gait(std::string name, GAIT_TYPE);

    ~Gait() {}

    void SetGaitPeriod(const float&);

    float GetStepHeight();

    void SetStepHeight(const float);

    void SetParams(float gait_period, float t0, float step_height, Eigen::Vector4d phi_thresh, Eigen::Vector4d phi_offset);

    void ShowParams();

    void updateInitTime(const float& t0); 

    Eigen::Array4i GetScheduledContact(const float& t);

    float GetStridePhase(const float& t, const int& leg_id);

    float GetSwingState(const float& t, const int& leg_id);

    float GetStanceState(const float& t, const int& leg_id);

    float GetStanceTime(const int& leg_id);

    float GetSwingTime(const int& leg_id);

    void UpdateStrideCount(int leg_id);

    int GetStrideCount(int leg_id);

    void ResetStrideCount();

    GAIT_TYPE GetGaitType();

    Eigen::Array4i m_n_stride = Eigen::Array4i::Zero();

private:
    void InitClass();

    std::string m_name;

    GAIT_TYPE m_gait_type;

    float m_t0;

    Eigen::Vector4d m_phi_thresh;

    Eigen::Vector4d m_phi_offset;

    Eigen::Array4i m_cs_ref_data;

    float m_gait_period;
    
    float m_step_height;
};

// @meshin: This overload is causing a duplicate symbol since it is not bound to any scope. Should fix this.
// std::ostream& operator<<(std::ostream& os, const Gait::GAIT_TYPE& gait_type) {
//     switch(gait_type) {
//         case Gait::STANCE:
//             os << "STANCE"; break;
//         case Gait::TROT:
//             os << "TROT"; break;
//         default:
//             os << "NOT SET";
//     }
// }