#include "Planner/Planner.hpp"

Planner::Planner() : m_name("svan_planner"), m_gait(Gait::TROT) {
    initClass();
}

Planner::Planner(std::string name) : m_name(name), m_gait(Gait::TROT) {
    initClass();
}

void Planner::initClass() {
    vec19 ee_init = m_robot.getStanceStates();
    if (m_sleep_start) {
        ee_init = m_robot.getSleepStates();
    }
    // Initialize the variables
    m_ee_state_ref = ee_init;
    m_ee_vel_ref = vec18::Zero();
    m_ee_acc_ref = vec18::Zero();

    m_p_takeoff_full = ee_init.block<12,1>(7, 0);

    m_ee_state_init = ee_init;

    p_cs_phi = Eigen::Vector4d::Zero();

    m_cs_act = int4::Ones();

    m_Pc_act = vec4::Zero();
    m_theta = vec12::Zero();
    m_joint_state_act = m_robot.getNeutralJointStates();
    m_joint_vel_act = vec18::Zero();

    m_t_curr = 0;
}

void Planner::startFromSleep(const bool& sleep_start) {
    m_sleep_start = sleep_start;
    initClass();
}

// Use this carefully! Might be a good idea to just remove it.
void Planner::setStance() {
    m_gait = Gait::GAIT_TYPE::STANCE;
    m_sleep_start = false;
    initClass();
    // m_ee_state_ref = m_robot.getStanceStates();
    // m_ee_vel_ref = vec18::Zero();
    // m_ee_acc_ref = vec18::Zero();
}

// void Planner::sleepToStance() {
// }

// Set desired base velocity in reference yaw rotated base frame
void Planner::setDesiredVelocity(const float& vx, const float& vy, const float& vyaw) {
    if (m_gait.GetGaitType() == Gait::GAIT_TYPE::STANCE && ((vx != 0) || (vy != 0) || (vyaw != 0))) {
        std::cout << "STANCE gait selected. Cannot set velocities.\n";
        std::cout << "Setting desired velocities to zero.\n";
        m_v_cmd_x = 0;
        m_v_cmd_y = 0;
        m_v_cmd_yaw = 0;
        return;
    }

    Eigen::Vector2d vel_cmd = Eigen::Vector2d(m_v_cmd_x, m_v_cmd_y);
    mat3x3 Rot_yaw = pinocchio::Rz(m_yaw);

    vec4 v_cmd = vec4::Zero();
    v_cmd.block<2,1>(0, 0) = Rot_yaw.block<2,2>(0, 0) * vel_cmd;
    v_cmd(2) = m_v_cmd_z;
    v_cmd(3) = m_v_cmd_yaw;    

    float kp = 2.0;
    float kd = 2.83;

    m_ee_acc_ref.block<3,1>(0, 0) = kd * (v_cmd.block<3,1>(0, 0) - m_ee_vel_ref.block<3,1>(0, 0));
    m_ee_acc_ref(5) = kd * (v_cmd(3) - m_ee_vel_ref(5));

    m_ee_acc_ref(0) = std::min(std::max(m_ee_acc_ref(0), -m_a_max_x), m_a_max_x);
    m_ee_acc_ref(1) = std::min(std::max(m_ee_acc_ref(1), -m_a_max_y), m_a_max_y);
    m_ee_acc_ref(2) = std::min(std::max(m_ee_acc_ref(2), -m_a_max_z), m_a_max_z);
    m_ee_acc_ref(5) = std::min(std::max(m_ee_acc_ref(5), -m_a_max_yaw), m_a_max_yaw);

    m_ee_vel_ref(0) += m_ee_acc_ref(0) * m_dt;
    m_ee_vel_ref(1) += m_ee_acc_ref(1) * m_dt;
    m_ee_vel_ref(2) += m_ee_acc_ref(2) * m_dt;
    m_ee_vel_ref(5) += m_ee_acc_ref(5) * m_dt;

    m_ee_vel_ref(0) = std::min(std::max(m_ee_vel_ref(0), -max_vel_x), max_vel_x);
    m_ee_vel_ref(1) = std::min(std::max(m_ee_vel_ref(1), -max_vel_y), max_vel_y);
    m_ee_vel_ref(2) = std::min(std::max(m_ee_vel_ref(2), -max_vel_z), max_vel_z);
    m_ee_vel_ref(5) = std::min(std::max(m_ee_vel_ref(5), -max_vel_yaw), max_vel_yaw);
}

bool Planner::setTargetBasePosition(const vec3& target_pos, const float& target_yaw) {

    float kp = 2.0;
    float kd = 3.0;

    m_ee_acc_ref.block<3,1>(0, 0) = kp * (target_pos - m_ee_state_ref.block<3,1>(0, 0)) + kd * (vec3::Zero() - m_ee_vel_ref.block<3,1>(0, 0));
    m_ee_acc_ref(5) = kp * (target_yaw - m_yaw) + kd * (0 - m_ee_vel_ref(5));

    m_ee_acc_ref(0) = std::min(std::max(m_ee_acc_ref(0), -m_a_max_x), m_a_max_x);
    m_ee_acc_ref(1) = std::min(std::max(m_ee_acc_ref(1), -m_a_max_y), m_a_max_y);
    m_ee_acc_ref(2) = std::min(std::max(m_ee_acc_ref(2), -m_a_max_z), m_a_max_z);
    m_ee_acc_ref(5) = std::min(std::max(m_ee_acc_ref(5), -m_a_max_yaw), m_a_max_yaw);

    m_ee_vel_ref(0) += m_ee_acc_ref(0) * m_dt;
    m_ee_vel_ref(1) += m_ee_acc_ref(1) * m_dt;
    m_ee_vel_ref(2) += m_ee_acc_ref(2) * m_dt;
    m_ee_vel_ref(5) += m_ee_acc_ref(5) * m_dt;

    m_ee_vel_ref(0) = std::min(std::max(m_ee_vel_ref(0), -max_vel_x), max_vel_x);
    m_ee_vel_ref(1) = std::min(std::max(m_ee_vel_ref(1), -max_vel_y), max_vel_y);
    m_ee_vel_ref(2) = std::min(std::max(m_ee_vel_ref(2), -max_vel_z), max_vel_z);
    m_ee_vel_ref(5) = std::min(std::max(m_ee_vel_ref(5), -max_vel_yaw), max_vel_yaw);

    vec3 base_position = m_ee_state_ref.block<3,1>(0, 0);
    vec3 delta_pos_pred = (target_pos - base_position);

    if ((delta_pos_pred).norm() < 1e-3) {
        // std::cout << "Target reached...\n";
        return true;
    }

    return false;
}

bool Planner::sleepToStance() {
    static float motion_start_time = m_t_curr;
    bool finished = false;

    m_gait = Gait::GAIT_TYPE::STANCE;
    vec19 ee_stance = m_robot.getStanceStates();

    float motion_timer = m_t_curr; // - motion_start_time;
    // std::cout << "Motion timer: " << motion_timer << "\n";
    // std::cout << "target pos: " << ee_stance.block<3,1>(0, 0).transpose() << "\n";
    float alpha = 0.999;
    if (motion_timer < 1.0) {
        m_ee_state_ref.block<12,1>(7, 0) = alpha * m_ee_state_ref.block<12,1>(7, 0) + (1 - alpha) * ee_stance.block<12,1>(7, 0);
    } else {
        finished = setTargetBasePosition(ee_stance.block<3,1>(0, 0), m_yaw);
    }

    if (finished) {
        motion_start_time = m_t_curr;
    }

    return finished;
}

bool Planner::stanceToSleep() {
    static float motion_start_time = m_t_curr;
    bool finished = false;

    m_gait = Gait::GAIT_TYPE::STANCE;
    setDesiredVelocity(0, 0, 0);

    vec19 ee_sleep = m_robot.getSleepStates();

    float motion_timer = m_t_curr - motion_start_time;

    finished = setTargetBasePosition(ee_sleep.block<3,1>(0, 0), m_yaw);


    // if (motion_timer < 4) {
    //     finished = setTargetBasePosition(ee_sleep.block<3,1>(0, 0), m_yaw, vec3(0, 0, 0.2));
    // } else {
    //     m_gait = Gait::GAIT_TYPE::STANCE;
    // }

    if (finished == true) {
        motion_start_time = m_t_curr;
    }

    return finished;
}

Planner::~Planner() {
}

void Planner::setBaseTarget() {
    float t = m_t_curr;

    m_ee_state_ref(0) += m_ee_vel_ref(0) * m_dt + 0.5 * m_ee_acc_ref(0) * m_dt * m_dt;
    m_ee_state_ref(1) += m_ee_vel_ref(1) * m_dt + 0.5 * m_ee_acc_ref(1) * m_dt * m_dt;
    m_ee_state_ref(2) += m_ee_vel_ref(2) * m_dt + 0.5 * m_ee_acc_ref(2) * m_dt * m_dt;

    vec3 eul = pinocchio::Rot2EulXYZ(pinocchio::quat2rot(m_ee_state_ref.block<4,1>(3, 0)));
    m_yaw = fmod(eul(2) + m_ee_vel_ref(5) * m_dt + 0.5 * m_ee_acc_ref(5) * m_dt * m_dt, 2 * M_PI);
    m_yaw = (m_yaw > M_PI) ? m_yaw - 2 * M_PI : m_yaw;
    vec3 ref_eul = vec3(0., 0., m_yaw);
    m_ee_state_ref.block<4,1>(3, 0) = pinocchio::EulXYZ2quat(ref_eul);
}

void Planner::updateTakeoffData() {
    Eigen::Array4i cs_ref_next = m_gait.GetScheduledContact(m_t_curr);

    // AM: m_cs_ref being updated in SetFeetTarget()

    for (int i = 0; i < 4; ++i) {

        if (cs_ref_next(i) == 0 && m_cs_ref(i) == 1) {
            // going from stance to swing phase
            m_t_takeoff(i) = m_t_curr;

            m_p_takeoff_full(3 * i) = m_ee_state_ref(7 + 3 * i);
            m_p_takeoff_full(3 * i + 1) = m_ee_state_ref(7 + 3 * i + 1);
        }
    }
}

vec3 Planner::getRaibertHeuristic(const uint8_t& leg_id, const float& t_stance) {
    Eigen::Vector3d p_step_i = Eigen::Vector3d::Zero();
    Eigen::VectorXd ee_ref = Eigen::VectorXd::Zero(19);
    Eigen::Vector2d base_ref_vel;

    // base_ref_vel << m_ee_ref.velocity[0], m_ee_ref.velocity[1];
    base_ref_vel << m_ee_vel_ref(0), m_ee_vel_ref(1);
    vec3 base_w_des = m_ee_vel_ref.block<3,1>(3, 0);

    ee_ref = m_ee_state_ref;
    vec3 base_pos_ref = m_ee_state_ref.block<3,1>(0, 0);

    vec19 js_ref = vec19::Zero();
    js_ref.block<7,1>(0, 0) = ee_ref.block<7,1>(0, 0);
    js_ref.block<12,1>(7, 0) = m_robot.inverseKinematics(ee_ref);

    vec12 p_hip = m_robot.getHFEPosition(js_ref);

    Eigen::Vector3d p_hip_i = p_hip.block<3, 1>(3 * leg_id, 0);

    vec3 r_hip_B = p_hip_i - base_pos_ref;
    Eigen::Vector3d ang_correction = base_w_des.cross(r_hip_B);

    p_step_i.block<2, 1>(0, 0) = p_hip_i.block<2, 1>(0, 0) + 0.5 * t_stance * ( base_ref_vel + ang_correction.block<2,1>(0, 0) );

    if (js_ref.hasNaN()) {
        std::cout << "dt: " << m_dt << "\n";
        std::cout << "t_curr: " << m_t_curr << "\n";
        std::cout << "ee_ref: " << ee_ref.transpose() << "\n";
        std::cout << "js_ref: " << js_ref.transpose() << "\n";
        std::cout << "p_hip: " << p_hip.transpose() << "\n";
        std::cout << "p_hip_i: " << leg_id << ": " << p_hip_i.transpose() << "\n";
        std::cout << "t_stance: " << t_stance << "\n";
        std::cout << "r_hip_B: " << r_hip_B.transpose() << "\n";
        std::cout << "p_step_i: " << leg_id << ": " << p_step_i.transpose() << "\n";
        std::cout << "leg_command: " << (p_step_i - m_p_takeoff_full.block<3,1>(3 * leg_id, 0)).transpose() << "\n";
        std::cout << "ang_correction: " << ang_correction.transpose() << "\n";
        exit(1);
    }

    return p_step_i;
}

Eigen::Vector3d Planner::getDesiredFootPosition(const uint8_t& leg_id, const float& t_stance) {
    return getRaibertHeuristic(leg_id, t_stance);
}

// // TODO: add z-height checks and print statements
// void Planner::CheckDesiredFootholds(Eigen::Vector3d& leg_command)
// {
//     float maxAngle = 0.7; // need to verify this limit
//     float maxDeviationFromNominal = m_robot.m_max_leg_extension * sin(maxAngle);

//         // Keep the foot from going too far from the body in +x
//         if (leg_command[0] > maxDeviationFromNominal)
//             leg_command[0] = maxDeviationFromNominal;

//         // Keep the foot from going too far from the body in -x
//         if (leg_command[0] < -maxDeviationFromNominal)
//             leg_command[0] = -maxDeviationFromNominal;

//         // Keep the foot from going too far from the body in +y
//         if (leg_command[1] > maxDeviationFromNominal)
//             leg_command[1] = maxDeviationFromNominal;

//         // Keep the foot from going too far from the body in -y
//         if (leg_command[1] < -maxDeviationFromNominal)
//             leg_command[1] = -maxDeviationFromNominal;

//         // Keep the leg under the motor module (don't raise above body or crash into
//         // module)
//         // if (data->_legController->commands[leg].pDes(2) >
//         //     -data->_quadruped->_maxLegLength / 4)
//         //     data->_legController->commands[leg].pDes(2) = -data->_quadruped->_maxLegLength / 4;

//         // // Keep the foot within the kinematic limits
//         // if (data->_legController->commands[leg].pDes(2) <
//         //     -data->_quadruped->_maxLegLength)
//         //     data->_legController->commands[leg].pDes(2) = -data->_quadruped->_maxLegLength;

// }

void Planner::setFeetTarget() {
    for (int i = 0; i < 4; ++i) {
        // if not in contact as per reference
        if (!m_cs_ref(i)) {
            // leg in swing phase, plan the swing trajectory
            Eigen::Vector3d p_step = getDesiredFootPosition(i, m_gait.GetStanceTime(i));

            Eigen::Vector3d p_takeoff = m_p_takeoff_full.block<3, 1>(3 * i, 0);
            // std::cout << "p_takeoff: " << p_takeoff.transpose() << "\n";

            Eigen::Vector3d leg_command = p_step -  p_takeoff;
            // std::cout << "leg command: " << leg_command.transpose() << "\n";
            // CheckDesiredFootholds(leg_command);

            // p_step = p_takeoff + leg_command;

            float t_takeoff = m_t_takeoff(i);
            float t_swing = m_gait.GetSwingTime(i);

            ref_foot_traj[i].setInitialPosition(p_takeoff.cast<float>());
            ref_foot_traj[i].setFinalPosition(p_step.cast<float>());
            ref_foot_traj[i].setHeight(m_gait.GetStepHeight());

            float swing_state = m_gait.GetSwingState(m_t_curr, i);

            ref_foot_traj[i].computeSwingTrajectoryBezier(swing_state, t_swing);

            Eigen::Vector3f leg_pos = ref_foot_traj[i].getPosition();
            Eigen::Vector3f leg_vel = ref_foot_traj[i].getVelocity();
            Eigen::Vector3f leg_acc = ref_foot_traj[i].getAcceleration();

            m_ee_state_ref(7 + 3 * i + 0) = leg_pos(0);
            m_ee_state_ref(7 + 3 * i + 1) = leg_pos(1);
            m_ee_state_ref(7 + 3 * i + 2) = leg_pos(2);

            m_ee_vel_ref(6 + 3 * i + 0) = leg_vel(0);
            m_ee_vel_ref(6 + 3 * i + 1) = leg_vel(1);
            m_ee_vel_ref(6 + 3 * i + 2) = leg_vel(2);

            m_ee_acc_ref(6 + 3 * i + 0) = leg_acc(0);
            m_ee_acc_ref(6 + 3 * i + 1) = leg_acc(1);
            m_ee_acc_ref(6 + 3 * i + 2) = leg_acc(2);
        }
    }
}

void Planner::setTarget() {
    setFeetTarget();
    setBaseTarget();
}

// bool Planner::CheckSafeOrientation(){
//     if (fabs(m_joint_state_act(3)) >= 0.5 || fabs(m_joint_state_act(4)) >= 0.5)
//     {
//         printf("Orientation safety check failed!\n");
//         return false;
//     } else {
//         return true;
//     }
// }

void Planner::updateExpectedStanceFlag() {
    m_expected_stance = int4::Zero();

    for (int leg_id = 0; leg_id < 4; leg_id++) {
        float phi_sw = m_gait.GetSwingState(m_t_curr, leg_id);

        if (m_cs_act(leg_id) && (phi_sw > 0.9)) {
            m_expected_stance(leg_id) = 1;
        } else {
            m_expected_stance(leg_id) = (phi_sw < 0);
        }
    }
}

// void Planner::adjustStanceLegs() {
//     // if a leg should be in stance but loses contact with ground, then move the foot down to regain contact
//     for (int leg_id = 0; leg_id < 4; leg_id++) {
//         if (m_expected_stance(leg_id) && !m_cs_act(leg_id)) {
//             // AM: this should ideally be terrain_ht(X, Y) - 0.01
//             printf("Adjusting leg %d to get in stance.", leg_id);
//             m_ee_ref.position[6 + 3 * leg_id + 2] = -0.01;
//         }
//     }
// }

Eigen::Vector4d Planner::getScheduledContactProbability(const float &t, Gait &gait)
{
    // variance in the subphase value when contact actually occurs (refer RPC thesis)
    Eigen::Array2f sigma_sq;
    sigma_sq << 0.05, 0.05;

    // compute the virtual support polygon for each foot
    Eigen::Vector4d cs_prob = Eigen::Vector4d::Zero();

    for (int i = 0; i < 4; i++)
    {
        // float phi = m_gait.GetStridePhase(t, i);
        float phi_sw = gait.GetSwingState(t, i);
        float phi_st = gait.GetStanceState(t, i);
        // in the following equations phi has been scaled appropriately to maintain the range from [0, 1] in both swing and stance
        float K_st = 0.5 * (erf(0.707 * phi_st / sqrt(sigma_sq[0])) +
                            erf(0.707 * (1 - phi_st) / sqrt(sigma_sq[1])));

        float K_sw = 0.5 * (2 + erf(-0.707 * phi_sw / sqrt(sigma_sq[0])) + erf(0.707 * (phi_sw - 1) / sqrt(sigma_sq[1])));

        int s_phi = m_cs_ref(i);
        cs_prob(i) = s_phi * K_st + (1 - s_phi) * K_sw;
    }

    return cs_prob;
}

double Planner::getTimeSinceStart() {
    auto currTimePont = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTimePoint).time_since_epoch().count();
    auto curr = std::chrono::time_point_cast<std::chrono::microseconds>(currTimePont).time_since_epoch().count();
    auto duration = curr - start;

    return duration * double(1e-6);
}

void Planner::reset() {
    m_startTimePoint = std::chrono::high_resolution_clock::now();

    m_gait = Gait::GAIT_TYPE::STANCE;
    setDesiredVelocity(0, 0, 0);

    if (m_planner_data_ptr == NULL) {
        std::cout << "Planner data pointer not set.\n";
        return;
    }

    m_planner_data_ptr -> x = m_robot.getStanceStates();
    m_planner_data_ptr -> xd = vec18::Zero();
    m_planner_data_ptr -> xdd = vec18::Zero();
    m_planner_data_ptr -> cs_ref = int4::Ones();
    m_planner_data_ptr -> pc_ref = vec4::Ones();

    initClass();
}

void Planner::step(const float& dt, const float& t_curr) {
    m_dt = dt;
    // std::cout << "m_dt: " << m_dt << "\n";
    m_t_curr = t_curr;
    // std::cout << "m_t_curr: " << m_t_curr << "\n";
    updateEstimationData();

    // Step 1
    updateTakeoffData();

    // Step 2
    m_cs_ref = m_gait.GetScheduledContact(m_t_curr);

    // Step 3
    updateExpectedStanceFlag();

    p_cs_phi = getScheduledContactProbability(m_t_curr, m_gait);

    // Step 4
    setTarget();

    updatePlannerData();
}

void Planner::updatePlannerData() {
    if (m_planner_data_ptr == NULL) {
        std::cout << "Planner data pointer not set. Returning empty\n";
        return;
    }
    m_planner_data_ptr -> x = m_ee_state_ref;
    m_planner_data_ptr -> xd = m_ee_vel_ref;
    m_planner_data_ptr -> xdd = m_ee_acc_ref;
    m_planner_data_ptr -> cs_ref = m_expected_stance;
    m_planner_data_ptr -> pc_ref = p_cs_phi;
}

void Planner::updateEstimationData() {
    if (m_estimation_data_ptr == NULL) {
        std::cout << "Estimation data pointer not set. Returning empty\n";
        return;
    }
    m_cs_act = m_estimation_data_ptr -> cs;
    m_Pc_act = m_estimation_data_ptr -> pc;
    m_theta = m_estimation_data_ptr -> js.block<12,1>(7, 0);
    m_joint_state_act = m_estimation_data_ptr -> js;
    m_joint_vel_act = m_estimation_data_ptr -> jv;
}