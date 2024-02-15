#pragma once

#include "Controller/Controller.hpp"

class DistributedIDC : public Controller {
public:
    DistributedIDC();
    DistributedIDC(const std::string& name);
    ~DistributedIDC() {}
private:
    void InitClass();
    std::string m_name;

    vec12 CalculateFeedForwardTorque() override;
    vec12 SolveContactForcesGPGD(const vec6 &);

    mat18x18 J, Jc, Jd, J_r, M, Nc, NcT, I;
    vec18 H, G, tau_full, qd_r, qdd_r, qd, a_cmd, PD;
    vec19 q_r, q;
    vec12 tau_ff;
    Eigen::VectorXd err;
    vec12 Fc, F_prev;
    vec6 b;

    mat6x6 Kpb, Kdb;
    mat12x12 Kpa, Kda;
};