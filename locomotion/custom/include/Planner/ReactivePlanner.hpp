#pragma once

#include "Planner/Planner.hpp"

class ReactivePlanner : public Planner {
public:
    ReactivePlanner();

    ReactivePlanner(std::string name);

    ~ReactivePlanner() {}

    vec3 GetDesiredFootPosition(const uint8_t&, const float&) override;

    void SetBaseTarget () override;

    void InitClass();
private:
    

    // std_msgs::Float32MultiArray m_p_cs_phi;
    Eigen::MatrixXd GetVPSPVertices(const float& t);
    double GetVPSPStabilityMargin();
    std::string m_name;

    bool use_vpsp = true;
    bool m_use_capture_point_heuristic = false;
};