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

    
    // float cutoff_freq = 2;
    // for (int i = 0; i < 12; ++i) {
    //     m_Fc_filter[i].setup(m_loop_rate, cutoff_freq);
    // }

    real_allocated = 0;
    num_vars_qp = 0;
    num_constr_qp = 0;
    c_st = 0;
    fz_min = 10;
    fz_max = 100;
}

void Controller::update_contact_flag_for_controller() {
    m_contact_flag_for_controller = int4::Zero();
    // for (int leg_id = 0; leg_id < 4; leg_id++) {
    //     // contact flag for controller is set as 1 iff the leg is (m_cs) and should be (m_expected_stance_flag) in contact
    //     if (m_expected_stance_flag(leg_id) && m_cs(leg_id))
    //         m_contact_flag_for_controller(leg_id) = 1;
    // }

    m_contact_flag_for_controller = m_cs;

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
    m_joint_command_ptr->kp = 60 * vec12::Ones();
    m_joint_command_ptr->kd = 5 * vec12::Ones();
}

vec12 Controller::GetDesiredContactForcePGD(const Eigen::MatrixXd& JabT, const vec6 &b) {
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
            // A.block<3, 3>(0, 3 * nc) = Eigen::MatrixXd::Identity(3, 3);
            // A.block<3, 3>(3, 3 * nc) = pinocchio::skew_symm(pf.block<3, 1>(3 * i, 0) - rc);
            A.block<6, 3>(0, 3 * nc) = JabT.block<6,3>(0, 3 * i);
            f_prev.block<3, 1>(3 * nc, 0) = F_prev.block<3, 1>(3 * i, 0);
            nc++;
        }
    }

    Eigen::VectorXd s(6);
    // s << 1, 1, 2, 10, 10, 5;
    s << 10, 10, 20, 100, 100, 20;
    // s << 1, 1, 1, 1, 1, 1;
    Eigen::MatrixXd S = s.asDiagonal();

    Eigen::VectorXd w = Eigen::VectorXd::Zero(3 * m_num_contact);

    for (int i = 0; i < m_num_contact; ++i) {
        w << 1, 1, 1;
    }

    // Eigen::MatrixXd W = 10 * Eigen::MatrixXd::Identity(3 * m_num_contact, 3 * m_num_contact);
    Eigen::MatrixXd W = 1e-1 * w.asDiagonal();
    Eigen::MatrixXd V = 1e-3 * Eigen::MatrixXd::Identity(3 * m_num_contact, 3 * m_num_contact);

    Eigen::MatrixXd Q = (A.transpose() * S * A + W + V);
    Eigen::MatrixXd P_T = -2 * (A.transpose() * S * b + V * f_prev);
    // Eigen::MatrixXd P = -b.transpose() * (S * A + A * S) - 2 * f_prev.transpose() * V;

    Eigen::VectorXd d_L = 2 * Q * f_prev + P_T;
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
    float mu = 0.8;
    float fn_max = 60;
    float fn_min = 0;

    float pf_n = 10;
    int max_iters = 1e3;
    int iters = 0;

    float alpha = 1e-2;

    // std::cout << "Problem setup passed.\n";

    while (iters <= max_iters)
    {
        d_L = 2 * Q * f + P_T;
        f = f - dd_L * d_L;
        // f = f - alpha * d_L;

        // projection on friction cone
        for (int i = 0; i < m_num_contact; i++) {
            f_t = f.block<2, 1>(3 * i, 0).norm();
            f_z = f(3 * i + 2);
            if (f_t <= mu * f_z) {
                pf_n = fminf(fmaxf(f_z, fn_min), fn_max);
                f_proj.block<2, 1>(3 * i, 0) = mu * pf_n * f.block<2, 1>(3 * i, 0) / f_t;
                f_proj(3 * i + 2) = pf_n;
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

        // if ((f - f_).norm() < 1e-3) {
        if (r.norm() < 5) {
            converged = true;
            break;
        }
        f_ = f;
        iters++;
    }

    // std::cout << "Residual: " << r.transpose() << std::endl;
    // std::cout << "b_hat: " << (A * f).transpose() << "\n";

    vec12 F = vec12::Zero();

    // // First order filtering
    float a = 0;

    nc = 0;
    for (int i = 0; i < 4; i++) {
        if (m_contact_flag_for_controller(i) == 0) {
        // if (m_cs(i) == 0) {
            F.block<3, 1>(3 * i, 0) = Eigen::Vector3d(0, 0, 0);
            continue;
        }
        // a = 1 - 0.1 * ()
        // F.block<3, 1>(3 * i, 0) = a * F_prev.block<3,1>(3 * i, 0) + (1 - a) * f.block<3, 1>(3 * nc++, 0);
        F.block<3, 1>(3 * i, 0) = f.block<3, 1>(3 * nc++, 0);
        // F(3 * i) = m_Fc_filter[3 * i].filter(f(3 * i));
        // F(3 * i + 1) = m_Fc_filter[3 * i + 1].filter(f(3 * i + 1));
        // F(3 * i + 2) = m_Fc_filter[3 * i + 2].filter(f(3 * i + 2));
    }

    // for (int i = 0; i < 12; ++i) {
    //     F(i) = m_Fc_filter[i].filter(F(i));
    // }

    
    // F = a * F_prev + (1 - a) * F;

    F_prev = F;

    return F;
}

// QP resizing
void Controller::resize_qpOASES_vars()
{
    if (real_allocated)
    {
        free(H_qpOASES);
        free(A_qpOASES);
        free(g_qpOASES);
        free(lbA_qpOASES);
        free(ubA_qpOASES);
        free(xOpt_qpOASES);
        free(xOpt_initialGuess);
    }

    H_qpOASES = (real_t *)malloc(num_vars_qp * num_vars_qp * sizeof(real_t));
    A_qpOASES = (real_t *)malloc(num_constr_qp * num_vars_qp * sizeof(real_t));
    g_qpOASES = (real_t *)malloc(num_vars_qp * 1 * sizeof(real_t));
    lbA_qpOASES = (real_t *)malloc(num_constr_qp * 1 * sizeof(real_t));
    ubA_qpOASES = (real_t *)malloc(num_constr_qp * 1 * sizeof(real_t));
    xOpt_qpOASES = (real_t *)malloc(num_vars_qp * 1 * sizeof(real_t));
    xOpt_initialGuess = (real_t *)malloc(num_vars_qp * 1 * sizeof(real_t));

    real_allocated = 1;
    // std::cout << "Resized QP vars." << std::endl;
}

// Eigen QP matrices resizing
void Controller::resize_eigen_vars()
{
    H_eigen.resize(num_vars_qp, num_vars_qp);
    A_eigen.resize(num_constr_qp, num_vars_qp);
    g_eigen.resize(num_vars_qp, 1);
    lbA_eigen.resize(num_constr_qp, 1);
    ubA_eigen.resize(num_constr_qp, 1);
    xOpt_eigen.resize(num_vars_qp, 1);

    H_eigen.setZero();
    A_eigen.setZero();
    g_eigen.setZero();
    lbA_eigen.setZero();
    ubA_eigen.setZero();
    xOpt_eigen.setZero();

    // std::cout << "Resized Eigen vars." << std::endl;
}

void Controller::update_problem_size()
{
    c_st = 0;
    for (int i = 0; i < 4; i++)
    {
        if (m_contact_flag_for_controller(i))
        // if (m_cs(i))
        {
            c_st++;
        }
    }

    num_vars_qp = 3 * c_st;
    num_constr_qp = 5 * c_st;
    // std::cout << "Updated problem size." << std::endl;
}

void Controller::copy_Eigen_to_real_t(real_t *target, Eigen::MatrixXd &source, int nRows, int nCols)
{
    int count = 0;

    // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not rows)
    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            target[count] = source(i, j);
            count++;
        }
    }
}

void Controller::copy_real_t_to_Eigen(Eigen::VectorXd &target, real_t *source, int len)
{
    for (int i = 0; i < len; i++)
    {
        target(i) = source[i];
    }
}

void Controller::print_real_t(real_t *matrix, int nRows, int nCols)
{
    int count = 0;
    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            std::cout << matrix[count] << "\t";
            count++;
        }
        std::cout << "\n";
    }
}

void Controller::print_QPData()
{
    std::cout << "\n\n";
    std::cout << "\n\nH = ";

    print_real_t(H_qpOASES, num_vars_qp, num_vars_qp);
    std::cout << "\n\nA = ";
    print_real_t(A_qpOASES, num_constr_qp, num_vars_qp);
    std::cout << "\n\ng = ";
    print_real_t(g_qpOASES, num_vars_qp, 1);
    std::cout << "\n\nlbA = ";
    print_real_t(lbA_qpOASES, num_constr_qp, 1);
    std::cout << "\n\nubA = ";
    print_real_t(ubA_qpOASES, num_constr_qp, 1);
}

Eigen::VectorXd Controller::clipVector(const Eigen::VectorXd &b, float F)
{
    Eigen::VectorXd result = b;
    for (int i = 0; i < result.size() / 2; ++i)
    {
        if (result[i] > F)
        {
            result[i] = F;
        }
        else if (result[i] < -F)
        {
            result[i] = -F;
        }
    }
    return result;
}

void Controller::cleanFc(vec12 &Fc)
{
    for (int i = 0; i < 4; i++)
    {
        if (Fc(3 * i + 2) < 0)
            Fc(3 * i + 2) = fz_min;

        if (Fc(3 * i + 2) > fz_max)
            Fc(3 * i + 2) = fz_max;
    }
}

// balance controller for the stance legs
vec12 Controller::getDesiredContactForceqpOASES(const vec6& b) {
    Eigen::VectorXd rc = m_joint_state_act.block<3, 1>(0, 0);          // CoM actual position
    Eigen::VectorXd pf = m_robot.forwardKinematics(m_joint_state_act); // computing the actual sim feet position in the inertial frame

    // std::cout << "b:" << b.lpNorm<Eigen::Infinity>() << std::endl;

    // update the QP matrices as per the current contact state
    update_problem_size();
    resize_qpOASES_vars();
    resize_eigen_vars();

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, num_vars_qp);
    Eigen::MatrixXd f_prev(num_vars_qp, 1);

    // compute the actual yaw rotation matrix
    // float yaw = m_joint_state_act(5);
    Eigen::Matrix3d R_yaw_act = Eigen::Matrix3d::Identity();

    int nc = 0;
    for (int i = 0; i < 4; i++)
    {
        if (m_contact_flag_for_controller(i))
        // if (m_cs(i))
        {
            A.block<3, 3>(0, 3 * nc) = R_yaw_act.transpose() * Eigen::MatrixXd::Identity(3, 3);
            A.block<3, 3>(3, 3 * nc) = R_yaw_act.transpose() * pinocchio::skew_symm(pf.block<3, 1>(3 * i, 0) - rc);
            f_prev.block<3, 1>(3 * nc, 0) = F_prev.block<3, 1>(3 * i, 0);
            nc++;
        }
    }

    Eigen::VectorXd s(6);
    // s << 1, 1, 5, 20, 20, 5;
    s << 1, 1, 5, 100, 100, 50;
    Eigen::MatrixXd S = s.asDiagonal();
    Eigen::MatrixXd W = 1e-0 * Eigen::MatrixXd::Identity(num_vars_qp, num_vars_qp);
    Eigen::MatrixXd V = 1e-3 * Eigen::MatrixXd::Identity(num_vars_qp, num_vars_qp);

    // CALCULATE H
    H_eigen = 2 * (A.transpose() * S * A + W + V);

    copy_Eigen_to_real_t(H_qpOASES, H_eigen, num_vars_qp, num_vars_qp);

    // CALCULATE g
    g_eigen = -2 * A.transpose() * S * b;
    g_eigen += -2 * V * f_prev;

    copy_Eigen_to_real_t(g_qpOASES, g_eigen, num_vars_qp, 1);

    // CALCULATE A
    // for now assuming mu is fixed, and n_i = [0 0 1], t1 = [1 0 0] and t2 = [0 1 0]
    Eigen::MatrixXd C_i(5, 3);
    C_i << 1, 0, -MU,
        0, 1, -MU,
        0, 1, MU,
        1, 0, MU,
        0, 0, 1;

    for (int i = 0; i < c_st; i++)
    {
        A_eigen.block<5, 3>(5 * i, 3 * i) = C_i;
    }

    copy_Eigen_to_real_t(A_qpOASES, A_eigen, num_constr_qp, num_vars_qp);

    // CALCULATE lbA and ubA
    Eigen::VectorXd di_lb(5);
    Eigen::VectorXd di_ub(5);

    di_lb << NEGATIVE_NUMBER,
        NEGATIVE_NUMBER,
        0,
        0,
        fz_min;

    di_ub << 0,
        0,
        POSITIVE_NUMBER,
        POSITIVE_NUMBER,
        fz_max;

    for (int i = 0; i < c_st; i++)
    {
        lbA_eigen.block<5, 1>(5 * i, 0) = di_lb;
        ubA_eigen.block<5, 1>(5 * i, 0) = di_ub;
    }

    copy_Eigen_to_real_t(lbA_qpOASES, lbA_eigen, num_constr_qp, 1);
    copy_Eigen_to_real_t(ubA_qpOASES, ubA_eigen, num_constr_qp, 1);

    // update the previous time-step's data in the initial guess
    copy_Eigen_to_real_t(xOpt_initialGuess, f_prev, num_vars_qp, 1);

    // solve the QP only if there is at least a leg in stance
    if (num_vars_qp)
    {
        QProblem qp_obj(num_vars_qp, num_constr_qp);
        Options options;
        options.setToMPC();
        options.printLevel = PL_NONE;
        qp_obj.setOptions(options);

    // print_QPData();

    nWSR_qpOASES = 1000;
    qp_exit_flag = qp_obj.init(
        H_qpOASES, g_qpOASES, A_qpOASES, nullptr, nullptr, lbA_qpOASES,
        ubA_qpOASES, nWSR_qpOASES, &cpu_time);

    std::cout << "Exit flag: " << qp_exit_flag << std::endl;
    // std::cout << "num_vars_qp: " << unsigned(num_vars_qp) << std::endl;

    qp_obj.getPrimalSolution(xOpt_qpOASES);
    copy_real_t_to_Eigen(xOpt_eigen, xOpt_qpOASES, num_vars_qp);
    }

    Eigen::VectorXd F = Eigen::VectorXd(12);

    nc = 0;
    for (int i = 0; i < 4; i++)
    {
        if (m_contact_flag_for_controller(i) == 0)
        // if (m_cs(i) == 0)
        {
            F.block<3, 1>(3 * i, 0) = Eigen::Vector3d(0, 0, 0);
            continue;
        }
        F.block<3, 1>(3 * i, 0) = xOpt_eigen.block<3, 1>(3 * nc++, 0);
    }

    F_prev = F;

    return F; // this is in inertial frame
}

Controller::~Controller() {
    // set zero torques, vel and init ee_state
    m_joint_command_ptr -> q = m_joint_state_init.block<12,1>(7, 0);
    m_joint_command_ptr -> qd = vec12::Zero();
    m_joint_command_ptr -> tau = vec12::Zero();

    updateJointCommand();

    std::cout << "Shutting down the controller. Sending default commands.\n";
}
