#include "Planner/Planner.hpp"

Planner::Planner() : m_name("svan_planner"), m_gait(Gait::TROT) {
    InitClass();
}

Planner::Planner(std::string name) : m_name(name), m_gait(Gait::TROT) {
    InitClass();
}

void Planner::InitClass() {
    vec19 ee_stance = m_robot.getStanceStates();
    // Initialize the variables
    m_ee_state_ref = m_robot.getStanceStates();
    m_ee_vel_ref = vec18::Zero();
    m_ee_acc_ref = vec18::Zero();

    m_p_takeoff_full = m_robot.getStanceStates().block<12,1>(7, 0);

    m_ee_state_init = ee_stance;

    // std::cout << "p_takeoff_init: " << m_p_takeoff_full.transpose() << "\n";

    p_cs_phi = Eigen::Vector4d::Zero();

    m_cs_act = int4::Ones();

    m_Pc_act = vec4::Zero();
    m_theta = vec12::Zero();
    m_joint_state_act = m_robot.getNeutralJointStates();
    m_joint_vel_act = vec18::Zero();

    reset();
}

void Planner::SetDesiredVelocity(const float& vx, const float& vy, const float& vyaw) {
    if (m_gait.GetGaitType() == Gait::GAIT_TYPE::STANCE) {
        std::cout << "STANCE gait selected. Cannot set velocities.\n";
        std::cout << "Setting desired velocities to zero.\n";
        return;
    }
    m_v_cmd_x = vx;
    m_v_cmd_y = vy;
    m_v_cmd_yaw = vyaw;
}

Planner::~Planner() {
    // // publish the initial reference values and call ros::shutdown()
    // svan_msgs::Trajectory ee_init;
    // ee_init.position.clear();
    // ee_init.velocity.clear();
    // ee_init.acceleration.clear();

    // for (int i = 0; i < 18; i++) {
    //     ee_init.position.push_back(m_ee_state_init(i));
    //     ee_init.velocity.push_back(0);
    //     ee_init.acceleration.push_back(0);
    // }
    // std::cout << "Sending default stance commands!\n";
}

void Planner::SetBaseTarget() {
    vec3 v_cmd = vec3::Zero();
    vec3 acc_cmd = vec3::Zero();
    float t = m_t_curr;

    v_cmd(0) = std::min(m_v_cmd_x, max_vel_x);
    v_cmd(1) = std::min(m_v_cmd_y, max_vel_y);
    v_cmd(2) = std::min(m_v_cmd_yaw, max_vel_yaw);

    // @meshin: Set the maximum acceleration as a fraction/multiple of the commanded velocity 
    acc_cmd = 0.8 * v_cmd;

    if (v_cmd(0) != 0) {
        // updating reference X commands
        float t_acc = v_cmd(0) / acc_cmd(0);
        // constant acceleration phase
        if (t <= t_acc)
        {
            // m_ee_ref.acceleration[0] = acc_cmd(0);
            // m_ee_ref.velocity[0] = acc_cmd(0) * t;
            // m_ee_ref.position[0] = 0.5 * acc_cmd(0) * t * t;
            m_ee_acc_ref(0) = acc_cmd(0);
            m_ee_vel_ref(0) = acc_cmd(0) * t;
            m_ee_state_ref(0) = 0.5 * acc_cmd(0) * t * t;
        }
        // constant velocity phase
        else
        {
            // m_ee_ref.acceleration[0] = 0;
            // m_ee_ref.velocity[0] = v_cmd(0);
            // m_ee_ref.position[0] = 0.5 * acc_cmd(0) * t_acc * t_acc + v_cmd(0) * (t - t_acc);
            m_ee_acc_ref(0) = 0;
            m_ee_vel_ref(0) = v_cmd(0);
            m_ee_state_ref(0) = 0.5 * acc_cmd(0) * t_acc * t_acc + v_cmd(0) * (t - t_acc);
        }
    }

    if (v_cmd[1] != 0)
    {
        // updating reference Y commands
        float t_acc = v_cmd(1) / acc_cmd(1);
        // constant acceleration phase
        if (t <= t_acc)
        {
            // m_ee_ref.acceleration[1] = acc_cmd(1);
            // m_ee_ref.velocity[1] = acc_cmd(1) * t;
            // m_ee_ref.position[1] = 0.5 * acc_cmd(1) * t * t;
            m_ee_acc_ref(1) = acc_cmd(1);
            m_ee_vel_ref(1) = acc_cmd(1) * t;
            m_ee_state_ref(1) = 0.5 * acc_cmd(1) * t * t;
        }
        // constant velocity phase
        else
        {
            // m_ee_ref.acceleration[1] = 0;
            // m_ee_ref.velocity[1] = v_cmd(1);
            // m_ee_ref.position[1] = 0.5 * acc_cmd(1) * t_acc * t_acc + v_cmd(1) * (t - t_acc);
            m_ee_state_ref(1) = 0;
            m_ee_vel_ref(1) = v_cmd(1);
            m_ee_state_ref(1) = 0.5 * acc_cmd(1) * t_acc * t_acc + v_cmd(1) * (t - t_acc);
        }
    }

    // m_ee_ref.acceleration[5] = 0;
    // m_ee_ref.velocity[5] = v_cmd(2);
    m_ee_acc_ref(5) = 0;
    m_ee_vel_ref(5) = v_cmd(2);
    float yaw = fmod(v_cmd(2) * t, 2 * M_PI);
    // Convert orientation to quaternion then pass it to m_ee_ref.position[3:7]
    // m_ee_ref.position[5] = (yaw > M_PI) ? yaw - 2 * M_PI : yaw;
}

void Planner::UpdateTakeoffData() {
    Eigen::Array4i cs_ref_next = m_gait.GetScheduledContact(m_t_curr);

    // AM: m_cs_ref being updated in SetFeetTarget()

    for (int i = 0; i < 4; ++i) {

        if (cs_ref_next(i) == 0 && m_cs_ref(i) == 1) {
            // going from stance to swing phase
            m_t_takeoff(i) = m_t_curr;

            // using the reference feet position
            // m_p_takeoff_full(3 * i) = m_ee_ref.position[7 + 3 * i];
            // m_p_takeoff_full(3 * i + 1) = m_ee_ref.position[7 + 3 * i + 1];
            m_p_takeoff_full(3 * i) = m_ee_state_ref(7 + 3 * i);
            m_p_takeoff_full(3 * i + 1) = m_ee_state_ref(7 + 3 * i + 1);

            // using the actual feet position
            // vec12 pf_act = m_robot.forwardKinematics(m_joint_state_act);
            // m_p_takeoff_full.block<2, 1>(3 * i, 0) = pf_act.block<2, 1>(3 * i, 0);
        }
    }
}

Eigen::Vector3d Planner::GetDesiredFootPosition(const uint8_t& leg_id, const float& t_stance) {
    Eigen::Vector3d p_step_i = Eigen::Vector3d::Zero();
    Eigen::VectorXd ee_ref = Eigen::VectorXd::Zero(19);
    Eigen::Vector2d base_ref_vel;

    // base_ref_vel << m_ee_ref.velocity[0], m_ee_ref.velocity[1];
    base_ref_vel << m_ee_vel_ref(0), m_ee_vel_ref(1);

    // for (int i = 0; i < 19; ++i) {
    //     // ee_ref(i) = m_ee_ref.position[i];
    //     ee_ref(i) = m_ee_state_ref(i);
    // }
    ee_ref = m_ee_state_ref;

    vec19 js_ref = vec19::Zero();
    js_ref.block<7,1>(0, 0) = ee_ref.block<7,1>(0, 0);
    js_ref.block<12,1>(7, 0) = m_robot.inverseKinematics(ee_ref);

    vec12 p_hip = m_robot.getHFEPosition(js_ref);

    Eigen::Vector3d p_hip_i = p_hip.block<3, 1>(3 * leg_id, 0);

    p_step_i.block<2,1>(0, 0) = p_hip_i.block<2,1>(0, 0) + 0.5 * t_stance * base_ref_vel;

    return p_step_i;
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

void Planner::SetFeetTarget() {

    for (int i = 0; i < 4; ++i) {
        // if not in contact as per reference
        if (!m_cs_ref(i)) {
            // leg in swing phase, plan the swing trajectory
            Eigen::Vector3d p_step = GetDesiredFootPosition(i, m_gait.GetStanceTime(i));

            Eigen::Vector3d p_takeoff = m_p_takeoff_full.block<3, 1>(3 * i, 0);

            Eigen::Vector3d leg_command = p_step -  p_takeoff;
            // CheckDesiredFootholds(leg_command);

            p_step = p_takeoff + leg_command;

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

            // m_ee_ref.position[7 + 3 * i + 0] = leg_pos(0);
            // m_ee_ref.position[7 + 3 * i + 1] = leg_pos(1);
            // m_ee_ref.position[7 + 3 * i + 2] = leg_pos(2);
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

void Planner::SetTarget() {
    SetFeetTarget();
    SetBaseTarget();
}

// bool Planner::CheckSafeOrientation(){
//     if (abs(m_joint_state_act(3)) >= 0.5 || abs(m_joint_state_act(4)) >= 0.5)
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
}

void Planner::Step() {
    updateEstimationData();

    m_t_curr = getTimeSinceStart();

    // std::cout << "t_curr: " << m_t_curr << std::endl;
    // reset if simulation restarted
    if (m_t_curr < 0) {
        reset();
    }

    // Step 1
    UpdateTakeoffData();

    // Step 2
    m_cs_ref = m_gait.GetScheduledContact(m_t_curr);

    // Step 3
    updateExpectedStanceFlag();
    // m_int32msg = copyArrayXiToMultiArray(m_expected_stance);

    p_cs_phi = getScheduledContactProbability(m_t_curr, m_gait);

    // Step 4
    SetTarget();

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