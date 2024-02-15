#include "Estimator/QuadEstimator.hpp"

QuadEstimator::QuadEstimator() : m_name("svan"), Estimator("svan") {
    InitClass();
}

QuadEstimator::QuadEstimator(std::string name) : m_name(name), Estimator(name) {
    InitClass();
}

void QuadEstimator::InitClass() {
    // Variable initialization

    m_forces_estimated = vec12::Zero();

    // m_sigma_pz = 0.841 * Eigen::Matrix4d::Identity(); // value from contact event detection paprt MIT
    m_sigma_pz = 2.0 * mat4x4::Identity(); // value from contact event detection paprt MIT
    // m_sigma_fz = 0.930 * Eigen::Matrix4d::Identity(); // value from contact event detection paprt MIT
    m_sigma_fz = 1.0 * mat4x4::Identity(); // value from contact event detection paprt MIT
    m_sigma_measurement = mat8x8::Zero();
    m_sigma_measurement.block<4,4>(0, 0) = m_sigma_pz;
    m_sigma_measurement.block<4,4>(4, 4) = m_sigma_fz;
    m_sigma_process = 1.5 * Eigen::Matrix4d::Identity();

    InitEstimatorVars();
}

void QuadEstimator::InitEstimatorVars() {
    float dt = m_dt;

    sensor_noise_zfoot          =  0.001;
    sensor_noise_pimu_rel_foot  =  0.001;
    sensor_noise_vimu_rel_foot  =  0.1;
    process_noise_pimu          =  0.02;
    process_noise_vimu          =  0.02;
    process_noise_pfoot         =  0.002;

    // Directly taken from cheetah source code
    _xhat.setZero();
    _ps.setZero();
    _vs.setZero();

    _A.setZero();
    _A.block<3,3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block<3,3>(0, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    _A.block<3,3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block<12,12>(6, 6) = Eigen::Matrix<double, 12, 12>::Identity();

    _B.setZero();
    _B.block<3,3>(0, 0) = 0.5 * dt * dt * Eigen::Matrix<double, 3, 3>::Identity();
    _B.block<3,3>(3, 0) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
    C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
    C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();

    _C.setZero();
    for (int i = 0; i < 4; ++i) _C.block<3,6>(3 * i, 0) = C1;
    _C.block<12,12>(0, 6) = double(-1) * Eigen::Matrix<double, 12, 12>::Identity();
    for (int i = 0; i < 4; ++i) _C.block<3,6>(12 + 3 * i, 0) = C2;
    _C(27, 17) = double(1);
    _C(26, 14) = double(1);
    _C(25, 11) = double(1);
    _C(24, 8) = double(1);

    _P.setIdentity();
    _P = 100. * _P;

    _Q0.setIdentity();
    _Q0.block<3,3>(0, 0) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block<3,3>(3, 3) =
        (dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block<12,12>(6, 6) = dt * Eigen::Matrix<double, 12, 12>::Identity();
    _R0.setIdentity();

    Q = Eigen::Matrix<double, 18, 18>::Identity();
    R = Eigen::Matrix<double, 28, 28>::Identity();

    Q.block<3,3>(0, 0) = _Q0.block<3,3>(0, 0) * process_noise_pimu;
    Q.block<3,3>(3, 3) = _Q0.block<3,3>(3, 3) * process_noise_vimu;
    Q.block<12,12>(6, 6) = _Q0.block<12,12>(6, 6) * process_noise_pfoot;

    R.block<12,12>(0, 0) = _R0.block<12,12>(0, 0) * sensor_noise_pimu_rel_foot;
        R.block<12,12>(12, 12) =
        _R0.block<12,12>(12, 12) * sensor_noise_vimu_rel_foot;
    R.block<4,4>(24, 24) = _R0.block<4,4>(24, 24) * sensor_noise_zfoot;
}

void QuadEstimator::UpdatePoseEstimate() {
    Q = Eigen::Matrix<double, 18, 18>::Identity();
    R = Eigen::Matrix<double, 28, 28>::Identity();

    Q.block<3,3>(0, 0) = _Q0.block<3,3>(0, 0) * process_noise_pimu;
    Q.block<3,3>(3, 3) = _Q0.block<3,3>(3, 3) * process_noise_vimu;
    Q.block<12,12>(6, 6) = _Q0.block<12,12>(6, 6) * process_noise_pfoot;

    R.block<12,12>(0, 0) = _R0.block<12,12>(0, 0) * sensor_noise_pimu_rel_foot;
        R.block<12,12>(12, 12) =
        _R0.block<12,12>(12, 12) * sensor_noise_vimu_rel_foot;
    R.block<4,4>(24, 24) = _R0.block<4,4>(24, 24) * sensor_noise_zfoot;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;

    // vec3 g(0, 0, double(-9.81));
    vec3 g(0, 0, 0);
    
    // mat3x3 Rot = pinocchio::quat2rot(sensor_data.quat);
    vec3 a_world = sensor_data.a_W;
    vec3 w_world = sensor_data.w_W;

    vec3 a = a_world + g;
    // std::cout << "A WORLD\n" << a << "\n";
    vec4 pzs = vec4::Zero();
    vec4 trusts = vec4::Zero();
    vec3 p0, v0;
    p0 << _xhat[0], _xhat[1], _xhat[2];
    v0 << _xhat[3], _xhat[4], _xhat[5];

    vec19 js_orig = m_robot.getNeutralJointStates();
    js_orig.block<4,1>(3, 0) = sensor_data.quat;
    js_orig.block<12,1>(7, 0) = sensor_data.q;
    vec18 jv_orig = vec18::Zero();
    jv_orig.block<3,1>(3, 0) = w_world;
    jv_orig.block<12,1>(6, 0) = sensor_data.qd;

    vec12 rFB = m_robot.forwardKinematics(js_orig);
    vec12 vFB = (m_robot.feetJacobian(js_orig) * jv_orig).block<12,1>(6, 0);

    for (int i = 0; i < 4; i++) {
        int i1 = 3 * i;

        vec3 p_f = rFB.block<3,1>(i1, 0);
        vec3 dp_f = vFB.block<3,1>(i1, 0);

        qindex = 6 + i1;
        rindex1 = i1;
        rindex2 = 12 + i1;
        rindex3 = 24 + i;

        double trust = double(1);
        // Using Contact probability as phase
        // double phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), double(1));
        double phase = est_data.pc(i);
        //double trust_window = double(0.25);
        double trust_window = double(0.2);

        if (phase < trust_window) {
            trust = phase / trust_window;
        } else if (phase > (double(1) - trust_window)) {
            trust = (double(1) - phase) / trust_window;
        }
        //double high_suspect_number(1000);
        double high_suspect_number(100);

        // printf("Trust %d: %.3f\n", i, trust);
        // Q.block(qindex, qindex, 3, 3) =
        //     (double(1) + (double(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
        Q.block<3,3>(qindex, qindex) =
            (double(1) + (double(1) - trust) * high_suspect_number) * Q.block<3,3>(qindex, qindex);

        // R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
        R.block<3,3>(rindex1, rindex1) = 1 * R.block<3,3>(rindex1, rindex1);
        // R.block(rindex2, rindex2, 3, 3) =
        //     (double(1) + (double(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
        R.block<3,3>(rindex2, rindex2) =
            (double(1) + (double(1) - trust) * high_suspect_number) * R.block<3,3>(rindex2, rindex2);
        // R(rindex3, rindex3) =
        //     (double(1) + (double(1) - trust) * high_suspect_number) * R(rindex3, rindex3);
        R(rindex3, rindex3) =
            (double(1) + (double(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

        trusts(i) = trust;

        _ps.segment(i1, 3) = -p_f;
        _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
        pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
    }

    Eigen::Matrix<double, 28, 1> y;
    y << _ps, _vs, pzs;
    _xhat = _A * _xhat + _B * a;
    Eigen::Matrix<double, 18, 18> At = _A.transpose();
    Eigen::Matrix<double, 18, 18> Pm = _A * _P * At + Q;
    Eigen::Matrix<double, 18, 28> Ct = _C.transpose();
    Eigen::Matrix<double, 28, 1> yModel = _C * _xhat;
    Eigen::Matrix<double, 28, 1> ey = y - yModel;
    Eigen::Matrix<double, 28, 28> S = _C * Pm * Ct + R;

    // todo compute LU only once
    Eigen::Matrix<double, 28, 1> S_ey = S.lu().solve(ey);
    _xhat += Pm * Ct * S_ey;

    Eigen::Matrix<double, 28, 18> S_C = S.lu().solve(_C);
    _P = (Eigen::Matrix<double, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

    Eigen::Matrix<double, 18, 18> Pt = _P.transpose();
    _P = (_P + Pt) / double(2);

    // std::cout << "_P: \n" << _P << "\n";
    // std::cout << "Before det(_P): " << _P.determinant() << "\n";
    // std::cout << "Before det(_P2x2): " << _P.block<2,2>(0, 0).determinant() << "\n";

    if (_P.block<2,2>(0, 0).determinant() > double(0.000001)) {
        _P.block<2,16>(0, 2).setZero();
        _P.block<16,2>(2, 0).setZero();
        _P.block<2,2>(0, 0) /= double(10);
    }
    // std::cout << "_P: \n" << _P << "\n";
    // std::cout << "After det(_P): " << _P.determinant() << "\n";
    // std::cout << "After det(_P2x2): " << _P.block<2,2>(0, 0).determinant() << "\n";

    // std::cout << "rB: " << _xhat.block<3,1>(0, 0).transpose() << "\n";
    // std::cout << "vB: " << _xhat.block<3,1>(3, 0).transpose() << "\n";
    // std::cout << "rP: " << _xhat.block<12,1>(6, 0).transpose() << "\n";

    est_data.rB = _xhat.block<3,1>(0, 0);
    est_data.vB = _xhat.block<3,1>(3, 0);
    est_data.rP = _xhat.block<12,1>(6, 0);
    // est_data.rB = base_position;
    // est_data.vB = base_vel;
    // vec19 js_tmp = m_robot.getNeutralJointStates();
    // js_tmp.block<3,1>(0, 0) = base_position;
    // js_tmp.block<4,1>(3, 0) = base_quat;
    // js_tmp.block<12,1>(7, 0) = sensor_data.q;
    // est_data.rP = m_robot.forwardKinematics(js_tmp);
}

inline float QuadEstimator::get_pc_pz(const float& pz, const float& mu_pz, const float & sigma_pz) {
    return 0.5 * (1 + std::erf((mu_pz - pz) / (sqrt(2) * sigma_pz)));
}
inline float QuadEstimator::get_pc_fz(const float& fz, const float& mu_fz, const float & sigma_fz) {
    return 0.5 * (1 + std::erf((fz - mu_fz) / (sqrt(2) * sigma_fz)));
}

vec4 QuadEstimator::get_pc_pz(const vec4& pz) {
    float mu_pz = 0.0;
    float sigma_pz = 0.1;
    vec4 m_pz_prob = vec4::Zero();
    for (int i = 0; i < 4; ++i) {
        m_pz_prob(i) = get_pc_pz(pz(i), mu_pz, sigma_pz);
    }
    return m_pz_prob;
}
vec4 QuadEstimator::get_pc_fz(const vec4& fz) {
    // mu_fz and sigma_fz can be made in to an indexable array to give different value for each foot
    float mu_fz = 15;
    float sigma_fz = 10;
    vec4 m_fz_prob = vec4::Zero();
    for (int i = 0; i < 4; ++i) {
        m_fz_prob(i) = get_pc_fz(fz(i), mu_fz, sigma_fz);
    }
    return m_fz_prob;
}

void QuadEstimator::UpdateContactEstimate() {
    vec19 js_tmp = m_robot.getNeutralJointStates();
    js_tmp.block<4,1>(3, 0) = sensor_data.quat;
    js_tmp.block<12,1>(7, 0) = sensor_data.q;

    mat18x18 jac = m_robot.feetJacobian(js_tmp);

    vec18 jv_tmp = vec18::Zero();
    jv_tmp.block<12,1>(6, 0) = sensor_data.qd;
    vec18 g = m_robot.gravitationalTerms(js_tmp);
    vec18 h = m_robot.nonLinearEffects(js_tmp, jv_tmp);
    m_forces_estimated = -jac.block<12,12>(6, 6).transpose().completeOrthogonalDecomposition().solve(sensor_data.tau - h.block<12,1>(6, 0));

    std::cout << "Fc_est: " << m_forces_estimated.transpose() << "\n";

    vec4 m_mu_Pc = m_pc_ref;
    mat4x4 m_sigma_Pc = m_sigma_process;

    // get the measurements and stack them
    vec4 pz = vec4::Zero();
    for (int i = 0; i < 4; ++i) {
        pz(i) = est_data.rP(3 * i + 2);
    }
    vec4 fz = vec4::Zero();
    for (int i = 0; i < 4; ++i) {
        fz(i) = m_forces_estimated(3 * i + 2);
    }

    // first measurement: P_c_pz (based on foot height estimate) (now probably pose estiamtor and this will feed each other. Hope things don't blow up!)
    vec4 pc_pz = get_pc_pz(pz);
    // second measurement: P_c_fz (based on contact foot force estimate)
    vec4 pc_fz = get_pc_fz(fz);

    // Measurement Update (Kalman)
    Eigen::MatrixXd Hk = Eigen::MatrixXd::Zero(8, 4);
    Hk.block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
    Hk.block<4, 4>(4, 0) = Eigen::Matrix4d::Identity();

    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(4, 8);
    K = m_sigma_Pc * Hk.transpose() * (Hk * m_sigma_Pc * Hk.transpose() + m_sigma_measurement).inverse();

    Eigen::VectorXd delta_zk = Eigen::VectorXd::Zero(8);
    delta_zk << (pc_pz - m_mu_Pc), (pc_fz - m_mu_Pc);

    // Posterior udpate
    m_mu_Pc = m_mu_Pc + K * delta_zk;
    m_sigma_Pc = (Eigen::Matrix4d::Identity() - K * Hk) * m_sigma_Pc;

    est_data.pc = m_mu_Pc;

    for (int i = 0; i < 4; ++i) {
        float P_c = m_mu_Pc(i);
        // if (P_c > 0.7 && m_cs_act(i) == 0) {
        //     m_cs_act(i) = 1;
        // } else if (P_c < 0.4 && m_cs_act(i) == 1) {
        //     m_cs_act(i) = 0;
        // }
        if (P_c > 0.5) {
            est_data.cs(i) = 1;
        } else {
            est_data.cs(i) = 0;
        }
    }
}

void QuadEstimator::ComputeEstimate() {
    UpdateContactEstimate();
    UpdatePoseEstimate();
}