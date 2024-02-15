#include <kinodynamics.hpp>

Quadruped::Quadruped() : urdf_filepath("/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/go2_description/urdf/go2_description.urdf") {
    InitClass();
}

Quadruped::Quadruped(std::string urdf_path) : urdf_filepath(urdf_path) {
    InitClass();
}

void Quadruped::InitClass() {
    pinocchio::urdf::buildModel(urdf_filepath, pinocchio::JointModelFreeFlyer(), model);
    pinocchio::Data tmp_data(model);
    data = tmp_data;

    std::vector<std::string> foot_name = {"FL", "FR", "RL", "RR"};
    std::string base_name = "base";

    if (model.existFrame(base_name)) {
        ID_BASE = model.getFrameId(base_name);
    } else {
        std::cerr << "Base frame: " << base_name << " missing from kinematic tree\n"; 
    }

    for (int i = 0; i < 4; ++i) {
        // Save foot frame IDs
        if (model.existFrame(foot_name[i] + "_foot")) {
            ID_FOOT_FRAME[i] = model.getFrameId(foot_name[i] + "_foot");
        } else {
            std::cerr << "Foot frame " << foot_name[i] + "_foot" << " missing from kinematic tree." << std::endl;
        }
        // Save foot joint IDs
        if (model.existFrame(foot_name[i] + "_foot_joint")) {
            ID_FOOT_JOINT[i] = model.getJointId(foot_name[i] + "_foot_joint");
        } else {
            std::cerr << "Foot Joint " << foot_name[i] + "_foot_joint" << " missing from kinematic tree." << std::endl;
        }
    }

    // Setting up robot parameters for IK

    this -> forwardKinematics(q_neutral);

    int ID_FR_FOOT = ID_FOOT_FRAME[1];

    // Modify these names as per URDF description
    for (int i = 0; i < 4; ++i) {
        ID_HAA[i] = model.getFrameId(foot_name[i] + "_hip_joint");
        ID_HFE[i] = model.getFrameId(foot_name[i] + "_thigh_joint");
        ID_KFE[i] = model.getFrameId(foot_name[i] + "_calf_joint");
    }
    ID_FL_HAA = model.getFrameId("FL_hip_joint");
    ID_FR_HAA = model.getFrameId("FR_hip_joint");
    ID_RL_HAA = model.getFrameId("RL_hip_joint");
    ID_RR_HAA = model.getFrameId("RR_hip_joint");

    ID_FL_HFE = model.getFrameId("FL_thigh_joint");
    ID_FR_HFE = model.getFrameId("FR_thigh_joint");
    ID_RL_HFE = model.getFrameId("RL_thigh_joint");
    ID_RR_HFE = model.getFrameId("RR_thigh_joint");

    ID_FL_KFE = model.getFrameId("FL_calf_joint");
    ID_FR_KFE = model.getFrameId("FR_calf_joint");
    ID_RL_KFE = model.getFrameId("RL_calf_joint");
    ID_RR_KFE = model.getFrameId("RR_calf_joint");
    

    h = (data.oMf[ID_FR_HAA].translation() - data.oMf[ID_RR_HAA].translation()).norm();
    b = (data.oMf[ID_FR_HAA].translation() - data.oMf[ID_FL_HAA].translation()).norm();

    l1 = (data.oMf[ID_FR_HAA].translation() - data.oMf[ID_FR_HFE].translation()).norm();
    l2 = (data.oMf[ID_FR_HFE].translation() - data.oMf[ID_FR_KFE].translation()).norm();
    l3 = (data.oMf[ID_FR_KFE].translation() - data.oMf[ID_FR_FOOT].translation()).norm();

    // std::cout << "h: " << h << "\n";
    // std::cout << "b: " << b << "\n";
    // std::cout << "l1: " << l1 << "\n";
    // std::cout << "l2: " << l2 << "\n";
    // std::cout << "l3: " << l3 << "\n";

    q_neutral = pinocchio::neutral(model);
    q_current = q_neutral;

    x_stance = vec19::Zero();
    x_stance.block<3,1>(0, 0) = vec3::Zero();
    x_stance(2) = l2 + l3 - 0.1;
    x_stance.block<4,1>(3, 0) = vec4::Zero();
    x_stance(6) = 1;
    for (int i = 0; i < 4; ++i) {
        x_stance.block<3,1>(7 + 3 * i, 0) = vec3(sx[i] * h/2, sy[i] * (b / 2 + l1), 0);
    }

    x_sleep = vec19::Zero();
    x_sleep.block<3,1>(0, 0) = vec3::Zero();
    x_sleep(2) = 0.1;
    x_sleep.block<4,1>(3, 0) = vec4::Zero();
    x_sleep(6) = 1;
    float sleep_y = 0.1;
    for (int i = 0; i < 4; ++i) {
        x_sleep.block<3,1>(7 + 3 * i, 0) = vec3(sx[i] * h/2, sy[i] * (b / 2 + l1 + sleep_y), 0);
    }
}

vec12 Quadruped::forwardKinematics(const vec19 & q) {
    vec12 feet_pos = vec12::Zero();

    pinocchio::framesForwardKinematics(model, data, q);

    for (int i = 0; i < 4; ++i) {
        feet_pos.block<3,1>(3*i, 0) = data.oMf[ID_FOOT_FRAME[i]].translation();
    }

    return feet_pos;
}

vec12 Quadruped::inverseKinematicsIterative(const vec19 & x) {
    vec19 q_res = q_current;

    vec12 feet_pos_trgt = x.block<12,1>(7, 0);
    pinocchio::SE3 base_des(pinocchio::quat2rot(x.block<4,1>(3, 0)), x.block<3,1>(0, 0));

    vec12 feet_pos_curr = this -> forwardKinematics(q_res);
    pinocchio::SE3 base_curr(pinocchio::quat2rot(q_res.block<4,1>(3, 0)), q_res.block<3,1>(0, 0));

    vec18 err;

    // while (true) {

    //     const pinocchio::SE3 d_base = data.oMf[ID_BASE].actInv(base_des);
    //     Eigen::VectorXd base_err = pinocchio::log6(d_base).toVector();

    //     feet_pos_curr = this -> forwardKinematics(q_res);
    //     Eigen::VectorXd feet_err = feet_pos_trgt - feet_pos_curr;

    //     err.block<6,1>(0, 0) = base_err;
    //     err.block<12,1>(0, 0) = feet_err;

    //     if (err.norm() < IK_ERR_TOL) {
    //         break;
    //     }
        
    //     if (iters > MAX_ITERS_IK) {
    //         std::cerr << "Max iterations exceeded. Potentially non-optimal solution. Solver failed.\n";
    //         std::cerr << "\nWarning: the iterative algorithm has not reached convergence to the desired precision\n";
    //         break;
    //     }
    //     Eigen::MatrixXd jac = this -> feetJacobian(q_res);
    //     jac = jac + DAMP * mat18x18::Identity();

    //     // pinocchio::Data::Matrix6 Jlog;
    //     pinocchio::Jlog6(d_base.inverse(), Jlog);
    //     // std::cout << "Jlog6: " << Jlog << std::endl;
    //     // jac.block<6,18>(0, 0) = -Jlog * jac.block<6,18>(0, 0);

    //     std::cout << "Jacobian det: " <<  jac.determinant() << std::endl;

    //     // Eigen::VectorXd dq = jac.inverse() * err;
    //     Eigen::VectorXd dq = jac.ldlt().solve(err);
    //     std::cout << "dq: " << dq.transpose() << std::endl;

    //     q_res = pinocchio::integrate(model, q_res, dq);

    //     iters++;        
    // }

    while (true) {
        feet_pos_curr = this -> forwardKinematics(q_res);
        Eigen::VectorXd feet_err = feet_pos_trgt - feet_pos_curr;

        std::cout << "err: " << feet_err.norm() << "\n";

        // err.block<6,1>(0, 0) = base_err;
        // err.block<12,1>(0, 0) = feet_err;

        if (feet_err.norm() < IK_ERR_TOL) {
            break;
        }
        
        if (iters > MAX_ITERS_IK) {
            std::cerr << "Max iterations exceeded. Potentially non-optimal solution. Solver failed.\n";
            std::cerr << "\nWarning: the iterative algorithm has not reached convergence to the desired precision\n";
            break;
        }
        Eigen::MatrixXd jac = this -> feetJacobian(q_res);
        Eigen::MatrixXd Jba = jac.block<12,12>(6, 0);
        // jac = jac + DAMP * mat18x18::Identity();

        // std::cout << "Jacobian det: " <<  jac.determinant() << std::endl;

        // Eigen::VectorXd dq = jac.inverse() * err;
        Eigen::VectorXd dq = Jba.completeOrthogonalDecomposition().solve(feet_err);
        std::cout << "dq: " << dq.transpose() << std::endl;

        // q_res = pinocchio::integrate(model, q_res, dq);
        q_res.block<12,1>(7, 0) -= dq;

        iters++;        
    }

    // return joint angles
    return q_res.block<12,1>(7, 0);
}

vec12 Quadruped::inverseKinematicsAnalytical(const vec19 & x) {
    // Analytical IK for 18-DoF quadruped (axes aligned with gazebo +ve axes)
    vec3 rB = x.block<3,1>(0, 0);
    mat3x3 R = pinocchio::quat2rot(x.block<4,1>(3, 0));
    vec12 feet_pos = x.block<12,1>(7, 0);

    vec12 joint_angles = vec12::Zero();

    for (int i = 0; i < 4; ++i) {
        vec3 r_HB = vec3(sx[i] * h/2, sy[i] * b/2, 0);
        vec3 rf = feet_pos.block<3,1>(3 * i, 0);
        vec3 r_fH = R.transpose() * (rf - rB) - r_HB;

        // std::cout << "r_B: " << rB.transpose() << std::endl;
        // std::cout << "r_f: " << rf.transpose() << std::endl;
        // std::cout << "r_HB: " << r_HB.transpose() << std::endl;
        // std::cout << "r_fH: " << r_fH.transpose() << std::endl;

        double x = r_fH(0);
        double y = r_fH(1);
        double z = r_fH(2);

        double et = y*y + z*z - l1*l1;

        // Calculation for theta3
        double c3 = (x*x + et - l2*l2 - l3*l3) / (2 * l2 * l3);
        double s3 = -std::sqrt(1 - c3*c3);

        double t3 = std::atan2(s3, c3);

        // Calculation for theta2
        double k1 = l2 + l3 * c3;
        double k2 = l3 * s3;

        double r1 = std::sqrt(k1*k1 + k2*k2);

        double t2 = std::atan2(-x/r1, std::sqrt(et) / r1) - atan2(k2/r1, k1/r1);

        // Calculation for theta1

        double zv = l2 * std::cos(t2) + l3 * std::cos(t2 + t3);
        double m1 = sy[i] * l1;
        double m2 = -zv;
        double r2 = std::sqrt(m1*m1 + m2*m2);

        double t1 = std::atan2(z/r2, y/r2) - std::atan2(m2/r2, m1/r2);

        joint_angles.block<3,1>(3*i, 0) = vec3(t1, t2, t3);
    }

    // TODO: Finally multiple by joint axis direction in URDF
    return joint_angles;
}

vec12 Quadruped::inverseKinematics(const vec19 & x, SOLVER_TYPE solver) {
    if (solver == SOLVER_TYPE::ITERATIVE) {
        std::cout << "\nBuggy solver. Use SOLVER_TYPE::ANALYTICAL for implementation.\n";
        return inverseKinematicsIterative(x);
    } else if (solver == SOLVER_TYPE::ANALYTICAL) {
        return inverseKinematicsAnalytical(x);
    }

    return vec12::Zero();
}

vec18 Quadruped::getJointVelocity(const vec19 & q, const vec18 & xd) {
    mat18x18 J = feetJacobian(q);
    vec18 qd;

    if (abs(J.determinant()) < 1e-15)
    {
        qd = vec18::Zero();
    }
    else
    {
        qd = J.completeOrthogonalDecomposition().solve(xd);
    }

    return qd;
}

mat18x18 Quadruped::feetJacobian(const vec19 & q) {
    mat18x18 feet_jac = mat18x18::Zero();

    // Required 2 calls before calling getFrameJacobian
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::framesForwardKinematics(model, data, q);

    // Get the jacobian for base
    pinocchio::getFrameJacobian(model, data, ID_BASE, pinocchio::LOCAL_WORLD_ALIGNED, feet_jac.block<6,18>(0, 0));

    // Get the Jacobian for the feet
    for (int i = 0; i < 4; ++i) {
        mat6x18 tmp_jac = mat6x18::Zero();
        // Use LOCAL_WORLD_ALIGNED and NOT WORLD. Read the below explanation of pinocchio::WORLD from the documentation
        // The WORLD frame convention corresponds to the frame concident with the Universe/Inertial frame but moving with the moving part (Joint, Frame, etc.).
        pinocchio::getFrameJacobian(model, data, ID_FOOT_FRAME[i], pinocchio::LOCAL_WORLD_ALIGNED, tmp_jac);
        feet_jac.block<3,18>(6 + 3 * i, 0) = tmp_jac.block<3, 18>(0, 0);
    }

    return feet_jac;
}

mat18x18 Quadruped::feetJacobianDot(const vec19& q, const vec18 & v) {
    mat18x18 jdot = mat18x18::Zero();

    pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);

    // Get the jacobian for base
    pinocchio::getFrameJacobianTimeVariation(model, data, ID_BASE, pinocchio::LOCAL_WORLD_ALIGNED, jdot.block<6,18>(0, 0));

    // Get the Jacobian for the feet
    for (int i = 0; i < 4; ++i) {
        mat6x18 tmp_jac = mat6x18::Zero();
        // Use LOCAL_WORLD_ALIGNED and NOT WORLD. Read the below explanation of pinocchio::WORLD from the documentation
        // The WORLD frame convention corresponds to the frame concident with the Universe/Inertial frame but moving with the moving part (Joint, Frame, etc.).
        pinocchio::getFrameJacobianTimeVariation(model, data, ID_FOOT_FRAME[i], pinocchio::LOCAL_WORLD_ALIGNED, tmp_jac);
        jdot.block<3,18>(6 + 3 * i, 0) = tmp_jac.block<3, 18>(0, 0);
    }

    return jdot;
}

mat18x18 Quadruped::massMatrix(const vec19 & q) {
    pinocchio::crba(model, data, q);
    // fill the lower triangle portion of the mass matrix
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    return data.M;
}

mat18x18 Quadruped::massMatrixInverse(const vec18 & q) {
    pinocchio::computeMinverse(model, data, q);

    return data.Minv;
}

vec18 Quadruped::coriolisVector(const vec19 & q, const vec18 & v) {
    return this -> nonLinearEffects(q, v) - this -> gravitationalTerms(q);
}

vec18 Quadruped::nonLinearEffects(const vec19 & q, const vec18 & v) {
    pinocchio::nonLinearEffects(model, data, q, v);
    return data.nle;
}

vec18 Quadruped::gravitationalTerms(const vec19 & q) {
    pinocchio::computeGeneralizedGravity(model, data, q);
    return data.g;
}

mat18x18 Quadruped::coriolisMatrix(const vec19 & q, const vec18 & v) {
    pinocchio::computeCoriolisMatrix(model, data, q, v);

    return data.C;
}

void Quadruped::setStanceHeight(const float& h) {
    x_stance(2) = h;
}

vec19 Quadruped::getStanceStates() {
    return x_stance;
}

vec19 Quadruped::getSleepStates() {
    return x_sleep;
}

vec19 Quadruped::getNeutralJointStates() {
    return q_neutral;
}

vec12 Quadruped::getHAAPosition(const vec19 & q) {
    vec12 haa_pos = vec12::Zero();

    pinocchio::framesForwardKinematics(model, data, q);

    for (int i = 0; i < 4; ++i) {
        haa_pos.block<3,1>(3*i, 0) = data.oMf[ID_HAA[i]].translation();
    }

    return haa_pos;
}

vec12 Quadruped::getHFEPosition(const vec19 & q) {
    vec12 hfe_pos = vec12::Zero();

    pinocchio::framesForwardKinematics(model, data, q);

    for (int i = 0; i < 4; ++i) {
        hfe_pos.block<3,1>(3*i, 0) = data.oMf[ID_HFE[i]].translation();
    }

    return hfe_pos;
}

// vec18 Quadruped::forwardDynamics(const vec18 & q, const vec18 & v, const vec12 & tau)
// {
//     return tau - coriolisVector(q, v) - gravitationalTerms(q);
// }

// Eigen::VectorXd Quadruped::inverseDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & a)
// {
//     return massMatrix(q)*a + coriolisVector(q, v) + gravitationalTerms(q);
// }