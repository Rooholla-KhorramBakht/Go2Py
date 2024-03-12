#pragma once

#include "Planner/Planner.hpp"

class ReactivePlanner : public Planner {
public:
    ReactivePlanner();

    ReactivePlanner(std::string name);

    ~ReactivePlanner() {}

    vec3 getDesiredFootPosition(const uint8_t&, const float&) override;

    // void SetBaseTarget () override;

    void setTarget() override;

    void initClass();
private:
    

    // std_msgs::Float32MultiArray m_p_cs_phi;
    Eigen::MatrixXd getVPSPVertices(const float& t);
    double getVPSPStabilityMargin();
    std::string m_name;

    bool use_vpsp = true;
    bool m_use_capture_point_heuristic = true;
};