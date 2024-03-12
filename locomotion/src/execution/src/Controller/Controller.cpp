#include "Controller/Controller.hpp"

Controller::Controller() : m_name("svan") {
    InitClass();
}

Controller::Controller(const std::string& name) : m_name(name) {
    InitClass();
}

void Controller::InitClass() {
    // std::string urdf_filepath = "/home/meshin/dev/quadruped/ws/src/unitree_ros/robots/" + m_name + "_description/urdf/" + m_name + "_description.urdf";
    // m_robot = Quadruped(urdf_filepath.c_str());

    m_joint_state_ref = vec19::Zero();
    m_joint_vel_ref = vec18::Zero();
    m_ff_torque = vec12::Zero();
    m_gait_id = 0;

    m_ee_state_init = vec19::Zero();
    m_ee_state_default_stance = vec19::Zero();

    // Variables
    m_ee_state_ref = m_robot.getStanceStates();
    m_ee_vel_ref = vec18::Zero();
    m_ee_acc_ref = vec18::Zero();

    m_joint_state_act = m_robot.getNeutralJointStates();
    m_joint_vel_act = vec18::Zero();
    m_joint_acc_act = vec18::Zero();

    m_state_error = vec19::Zero();
    m_eta = vec18::Zero();
    m_cs = int4::Zero();
    m_expected_stance_flag = int4::Ones();
    m_num_contact = 0;

    // m_ee_state_init = m_robot.getStanceStates();
    if (!m_starting_from_sleep) {
        // initial footholds in default stance, in global frame
        m_ee_state_init = m_robot.getStanceStates();
    } else {
        // initial footholds in default sleep, in global frame
        m_ee_state_init = m_robot.getSleepStates();
    }

    // m_ee_state_init(0) = -0.05;
    // m_ee_state_init(1) = 0.02;
    // m_ee_state_init(7 + 3 * 1 + 2) = 0.1;

    m_ee_state_ref = m_ee_state_init;
    // m_joint_state_init = m_robot.inverseKinematics(m_ee_state_init);

    // this variable is used to reset the robot in case of unsafe commands, it should go into stance, not sleep
    m_ee_state_default_stance = m_robot.getStanceStates();
    m_joint_state_init.block<7,1>(0, 0) = m_ee_state_default_stance.block<7,1>(0, 0);
    m_joint_state_init.block<12,1>(7, 0) = m_robot.inverseKinematics(m_ee_state_default_stance);

    m_contact_flag_for_controller = int4::Ones();

    m_joint_state_act = m_joint_state_init;
    m_joint_vel_act = vec18::Zero();

    // used in GetDesiredContactForcePGD()
    F_prev = vec12::Zero();
}

void Controller::update_contact_flag_for_controller() {
    m_contact_flag_for_controller = int4::Zero();
    for (int leg_id = 0; leg_id < 4; leg_id++) {
        // contact flag for controller is set as 1 iff the leg is (m_cs) and should be (m_expected_stance_flag) in contact
        if (m_expected_stance_flag(leg_id) && m_cs(leg_id))
            m_contact_flag_for_controller(leg_id) = 1;
    }

    m_num_contact = m_contact_flag_for_controller.sum();
}

void Controller::Step() {
    updateEstimationData();
    updatePlannerData();
    // Timer timer("Controller step");

    m_joint_state_ref.block<7,1>(0, 0) = m_ee_state_ref.block<7, 1>(0, 0);
    m_joint_state_ref.block<12,1>(7, 0) = m_robot.inverseKinematics(m_ee_state_ref);

    m_joint_vel_ref = m_robot.getJointVelocity(m_joint_state_ref, m_ee_vel_ref);

    update_contact_flag_for_controller();

    m_ff_torque = CalculateFeedForwardTorque();

    // std::cout << "Waah bc!\n";

    if (m_joint_state_ref.hasNaN()) {
        std::cout << "NaN encountered in states! Setting to default stance.\n";
        // std::cout << "IK called with ee states: \n"
        //           << m_ee_state_ref.transpose() << std::endl;
        // std::cout << "joint states: \n"
        //           << m_joint_state_ref.transpose() << std::endl;
        m_joint_state_ref = m_joint_state_init;
    }
    if (m_joint_vel_ref.hasNaN()) {
        std::cout << "NaN encountered in vel! Setting velocity to zero.\n";
        // std::cout << "joint vel: \n"
        //           << m_joint_vel_ref.transpose() << std::endl;
        m_joint_vel_ref = vec18::Zero(0);
    }
    if (m_ff_torque.hasNaN() || m_joint_state_act(3) >= 3 || m_joint_state_act(3) <= -3) {
        std::cout << "NaN encountered in torques! Setting torque to zero.\n";
        // std::cout << "ff torque: \n"
        //           << m_ff_torque.transpose() << std::endl;
        m_ff_torque = vec12::Zero();
    }
    if (m_joint_state_act(3) >= 1.57 || m_joint_state_act(3) <= -1.57) {
        std::cout << "Roll limit exceeded! Setting torque to zero.\n";
        // std::cout << "ff torque: \n"
        //           << m_ff_torque.transpose() << std::endl;
        m_ff_torque = vec12::Zero();
    }
    // do lie subtraction to get actual error vector
    // m_state_error = m_joint_state_ref - m_joint_state_act;

    // for (int i = 0; i < 18; ++i) {
    //     m_state_error_data.data[i] = m_state_error(i);
    // }
    updateJointCommand();
}

void Controller::updateEstimationData() {
    m_joint_state_act = m_estimation_data_ptr -> js;
    m_joint_vel_act = m_estimation_data_ptr -> jv;
    m_joint_acc_act = m_estimation_data_ptr -> ja;
    m_cs = m_estimation_data_ptr -> cs;
    // m_num_contact = m_cs.sum();
}

void Controller::updatePlannerData() {
    m_ee_state_ref = m_planner_data_ptr -> x;
    m_ee_vel_ref = m_planner_data_ptr -> xd;
    m_ee_acc_ref = m_planner_data_ptr -> xdd;
    m_expected_stance_flag = m_planner_data_ptr -> cs_ref;
}

void Controller::updateJointCommand() {
    m_joint_command_ptr->q = m_joint_state_ref.block<12,1>(7, 0);
    m_joint_command_ptr->qd = m_joint_vel_ref.block<12,1>(6, 0);
    m_joint_command_ptr->tau = m_ff_torque;
    m_joint_command_ptr->kp = 20 * vec12::Ones();
    m_joint_command_ptr->kd = 5 * vec12::Ones();
}

vec12 Controller::GetDesiredContactForcePGD(const vec6 &b) {
    // Timer timer("GPGD Step");
    // std::cout << "NUM CONTACT: " << m_num_contact << "\n";
    // std::cout << "m_cs: " << m_cs.transpose() << std::endl;
    // std::cout << "m_contact_flag_for_controller: " << m_contact_flag_for_controller.transpose() << std::endl;
    if (m_num_contact == 0 || b.hasNaN())
    {
        return vec12::Zero();
    }

    vec3 rc = m_joint_state_act.block<3, 1>(0, 0);          // CoM actual position
    vec12 pf = m_robot.forwardKinematics(m_joint_state_act); // computing the actual sim feet position in the inertial frame

    int nc = 0; // a counter

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 3 * m_num_contact);
    Eigen::VectorXd f_prev = Eigen::VectorXd::Zero(3 * m_num_contact);

    nc = 0;
    for (int i = 0; i < 4; i++)
    {
        if (m_contact_flag_for_controller(i)) {
        // if (m_cs(i)) {
            A.block<3, 3>(0, 3 * nc) = Eigen::MatrixXd::Identity(3, 3);
            A.block<3, 3>(3, 3 * nc) = pinocchio::skew_symm(pf.block<3, 1>(3 * i, 0) - rc);
            f_prev.block<3, 1>(3 * nc, 0) = F_prev.block<3, 1>(3 * i, 0);
            nc++;
        }
    }

    Eigen::VectorXd s(6);
    // s << 1, 1, 2, 10, 10, 5;
    s << 1, 1, 5, 100, 100, 50;
    Eigen::MatrixXd S = s.asDiagonal();
    Eigen::MatrixXd W = 1e-5 * Eigen::MatrixXd::Identity(3 * m_num_contact, 3 * m_num_contact);
    Eigen::MatrixXd V = 1e-3 * Eigen::MatrixXd::Identity(3 * m_num_contact, 3 * m_num_contact);

    Eigen::MatrixXd Q = (A.transpose() * S * A + W + V);
    Eigen::MatrixXd P = -2 * (A.transpose() * S * b + V * f_prev);

    Eigen::VectorXd d_L = 2 * Q * f_prev + P;
    Eigen::MatrixXd dd_L = 0.5 * Q.inverse();

    Eigen::VectorXd r;
    Eigen::Vector4d cv(0, 0, 0, 0);
    float violation = 0;
    Eigen::VectorXd f = Eigen::VectorXd(f_prev);
    Eigen::VectorXd f_ = 100 * Eigen::VectorXd::Ones(3 * m_num_contact);
    Eigen::VectorXd f_proj = Eigen::VectorXd(f_prev);

    bool converged = false;
    float f_t = 1;
    float f_z = 10;
    float mu = 0.2;
    float fn_max = 200;
    float fn_min = 5;

    float pf_n = 10;
    int max_iters = 1e2;
    int iters = 0;

    float alpha = 1e-2;

    // std::cout << "Problem setup passed.\n";

    while (iters <= max_iters)
    {
        d_L = 2 * Q * f + P;
        f = f - dd_L * d_L;
        // f = f - alpha * d_L;

        // projection on friction cone
        for (int i = 0; i < m_num_contact; i++) {
            f(3 * i + 2) = fminf(fmaxf(f(3 * i + 2), fn_min), fn_max);
            f_t = f.block<2, 1>(3 * i, 0).norm();
            f_z = f(3 * i + 2);

            f.block<2, 1>(3 * i, 0) = mu * f(3 * i + 2) * (f.block<2, 1>(3 * i, 0) / f_t);
            if (f_t <= mu * f_z) {
                f_proj.block<3, 1>(3 * i, 0) = f.block<3, 1>(3 * i, 0);
            }
            else if (f_t <= -f_z / mu) {
                f_proj.block<3, 1>(3 * i, 0) = Eigen::Vector3d(0, 0, 0);
            }
            else {
                pf_n = (f_t * mu + f_z) / (mu * mu + 1);
                pf_n = fminf(fmaxf(pf_n, fn_min), fn_max);
                f_proj.block<2, 1>(3 * i, 0) = mu * pf_n * f.block<2, 1>(3 * i, 0) / f_t;
                f_proj(3 * i + 2) = pf_n;
            }
        }

        f = f_proj;
        r = A * f - b;

        if ((f - f_).norm() < 1e-3) {
        // if ((f - f_).norm() < 1e-3 && r.norm() < 50) {
            converged = true;
            break;
        }
        f_ = f;
        iters++;
    }

    // std::cout << "Residual: " << r.transpose() << std::endl;

    vec12 F = vec12::Zero();

    // // First order filtering
    float a = 0.2;

    nc = 0;
    for (int i = 0; i < 4; i++) {
        if (m_contact_flag_for_controller(i) == 0) {
        // if (m_cs(i) == 0) {
            F.block<3, 1>(3 * i, 0) = Eigen::Vector3d(0, 0, 0);
            continue;
        }
        F.block<3, 1>(3 * i, 0) = a * F_prev.block<3,1>(3 * i, 0) + (1 - alpha) * f.block<3, 1>(3 * nc++, 0);
    }

    
    // F = a * F_prev + (1 - a) * F;

    F_prev = F;

    return F;
}

Controller::~Controller() {
    // set zero torques, vel and init ee_state
    m_joint_command_ptr -> q = m_joint_state_init.block<12,1>(7, 0);
    m_joint_command_ptr -> qd = vec12::Zero();
    m_joint_command_ptr -> tau = vec12::Zero();

    updateJointCommand();

    std::cout << "Shutting down the controller. Sending default commands.\n";
}
