#include "Controller/DistributedIDC.hpp"

DistributedIDC::DistributedIDC(const std::string& name) : m_name(name), Controller(name) {
    InitClass();
}

DistributedIDC::DistributedIDC() : m_name("svan"), Controller("svan") {
    InitClass();
}

void DistributedIDC::InitClass() {
    J = mat18x18::Zero();
    Jc = mat18x18::Zero();
    Jd = mat18x18::Zero();
    J_r = mat18x18::Zero();
    M = mat18x18::Zero();
    Nc = mat18x18::Zero();
    NcT = mat18x18::Zero();
    H = vec18::Zero();
    G = vec18::Zero();
    I = mat18x18::Identity();
    tau_full = vec18::Zero();
    tau_ff = vec12::Zero();
    q_r = vec19::Zero();
    qd_r = vec18::Zero();
    qdd_r = vec18::Zero();
    q = vec19::Zero();
    qd = vec18::Zero();
    PD = vec18::Zero();

    err = Eigen::VectorXd::Zero(36);
    // r = Eigen::VectorXd::Zero(18);
    Fc = vec12::Zero();
    F_prev = vec12::Zero();
    b = vec6::Zero();

    // load gains from a param file
    vec6 kpb;
    kpb << 316.22, 316.22, 26.22,
        316.22, 316.22, 26.22;
    // kpb << 0, 0, 26.22,
    //     316.22, 316.22, 26.26;
    // kpb << 0, 0, 26.22,
    // 316.22, 316.22, 316.22;
    Kpb = kpb.asDiagonal();

    vec6 kdb;
    kdb << 50.40, 50.40, 13.40,
        50.4, 50.4, 13.40;
    // kdb << 0 * 13.40, 0 * 13.40, 13.40,
    //     50.4, 50.4, 0 * 13.40;
    Kdb = kdb.asDiagonal();

    vec12 kpa;
    vec3 kp(26.26, 26.26, 26.26);
    // vec3 kp(10.26, 10.26, 10.26);
    kpa << kp, kp, kp, kp;
    Kpa = kpa.asDiagonal();

    vec12 kda;
    vec3 kd(13.03, 13.53, 13.53);
    // vec3 kd(1.4, 1.4, 1.4);
    kda << kd, kd, kd, kd;
    Kda = kda.asDiagonal();
}

vec12 DistributedIDC::CalculateFeedForwardTorque() {
    // Timer timer("DIDC loop time");
    q = m_joint_state_act;
    qd = m_joint_vel_act;
    q_r = m_joint_state_ref;
    qd_r = m_joint_vel_ref;
    
    // std::cout << "q: " << q.transpose() << "\n";
    // std::cout << "qd: " << qd.transpose() << "\n";
    // std::cout << "qr: " << q_r.transpose() << "\n";
    // std::cout << "qdr: " << qd_r.transpose() << "\n";

    {
        // Timer timer1("Pinocchio dynamics calls");
        M = m_robot.massMatrix(q);
        H = m_robot.coriolisVector(q, qd);
        G = m_robot.gravitationalTerms(q);
    }

    // std::cout << "M: \n" << M << "\n";
    // std::cout << "H: \n" << H.transpose() << "\n";
    // std::cout << "G: \n" << G.transpose() << "\n";

    vec6 base_err = vec6::Zero();
    base_err.block<3,1>(0, 0) = q_r.block<3,1>(0, 0) - q.block<3,1>(0, 0);
    mat3x3 R_ref = pinocchio::quat2rot(q_r.block<4,1>(3, 0));
    mat3x3 R_act = pinocchio::quat2rot(q.block<4,1>(3, 0));
    // log(R_ref.transpose() * R_act)
    base_err.block<3,1>(3, 0) = pinocchio::matrixLogRot(R_ref * R_act.transpose());
    // std::cout << "base err: " << base_err.transpose() << "\n";
    PD.block<6, 1>(0, 0) = 1 * Kpb * (base_err)
                         + 1 * Kdb * (qd_r.block<6, 1>(0, 0) - qd.block<6, 1>(0, 0));
    PD.block<12, 1>(6, 0) = 1 * Kpa * (q_r.block<12, 1>(7, 0) - q.block<12, 1>(7, 0))
                          + 1 * Kda * (qd_r.block<12, 1>(6, 0) - qd.block<12, 1>(6, 0));    
    // err.block<18,1>(0, 0) = q_r - q;
    // err.block<18,1>(18, 0) = qd_r - qd;

    mat18x18 J_r = mat18x18::Zero();
    mat18x18 Jd_r = mat18x18::Zero();
    
    {    
        // Timer timer1("Pinocchio Jacobians");
        J = m_robot.feetJacobian(q);
        J_r = m_robot.feetJacobian(q_r);
        Jd_r = m_robot.feetJacobianDot(q_r, qd_r);
    }
    

    // std::cout << "J: \n" << J << "\n";

    

    qdd_r = J_r.completeOrthogonalDecomposition().solve(m_ee_acc_ref - Jd_r * qd_r);

    // std::cout << "qdd_r: " << qdd_r.transpose() << "\n";

    a_cmd = qdd_r + PD;
    tau_full = M * (a_cmd) + H + G;

    // std::cout << "tau_full: " << tau_full.transpose() << "\n";

    // distributing the 18x1 generalized force into 12x1 joint space torques
    Eigen::MatrixXd JabT = J.block<12, 6>(6, 0).transpose();
    Eigen::MatrixXd Jaa = J.block<12, 12>(6, 6);
    Eigen::MatrixXd JaaT = Jaa.transpose();
    // std::cout << "distributing1" << std::endl;
    Eigen::MatrixXd JabT_inv = JabT.transpose() * (JabT * JabT.transpose()).inverse();
    // std::cout << "distributing2" << std::endl;
    // Map from body wrench to joint torques: should be -JaaT * JabT_inv
    Eigen::MatrixXd J_b2t = -JaaT * JabT_inv;
    Eigen::MatrixXd N_b2t = (Eigen::MatrixXd::Identity(12, 12) - J_b2t * (J_b2t.transpose() * J_b2t).inverse() * J_b2t.transpose());

    b.block<3,1>(0, 0) = tau_full.block<3,1>(0, 0);
    b.block<3,1>(3, 0) = tau_full.block<3,1>(3, 0);
    // Fc = getFeetForces(clipVector(b, 50));
    std::cout << "b: " << b.transpose() << "\n";
    // b(0) = std::min(std::max(b(0), -20.), 20.);
    // b(1) = std::min(std::max(b(1), -20.), 20.);
    // b(3) = std::min(std::max(b(3), -20.), 20.);
    // b(4) = std::min(std::max(b(4), -20.), 20.);
    // b(5) = std::min(std::max(b(5), -20.), 20.);
    // std::cout << "b_clipped: " << b.transpose() << "\n";
    Fc = GetDesiredContactForcePGD(b);
    std::cout << "cs: " << m_contact_flag_for_controller.transpose() << "\n";
    std::cout << "Fc: " << Fc.transpose() << std::endl;

    tau_ff = -JaaT * Fc + N_b2t * tau_full.block<12,1>(6, 0);
    // tau_ff = tau_full.block<12,1>(6, 0);

    #ifdef PRINT_DEBUG
        // std::cout << "Fc: " << Fc.transpose() << std::endl;
        // std::cout << "tau_ff: " << tau_ff.transpose() << std::endl;
    #endif

    return tau_ff;
}