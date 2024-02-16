#include "Planner/ReactivePlanner.hpp"
#include <cmath>

ReactivePlanner::ReactivePlanner() : Planner("svan"), m_name("svan_planner")
{
    InitClass();
}

ReactivePlanner::ReactivePlanner(std::string name) : Planner(name), m_name(name)
{
    InitClass();
}

void ReactivePlanner::InitClass() {
}

vec3 ReactivePlanner::GetDesiredFootPosition(const uint8_t &leg_id, const float &t_stance) {
    Eigen::Vector3d p_step_i = Eigen::Vector3d::Zero();
    vec19 ee_ref = vec19::Zero();
    Eigen::Vector2d base_ref_vel, base_vel_act;

    base_vel_act = m_joint_vel_act.block<2, 1>(0, 0);
    base_ref_vel << m_ee_vel_ref(0), m_ee_vel_ref(1);

    Eigen::Vector3d base_w_des = vec3::Zero();
    for (int i = 0; i < 3; ++i) base_w_des(i + 0) = m_ee_vel_ref(i + 3);

    ee_ref = m_ee_state_ref;

    vec19 js_ref = vec19::Zero();
    js_ref.block<7,1>(0, 0) = ee_ref.block<7,1>(0, 0);
    js_ref.block<12,1>(7, 0) = m_robot.inverseKinematics(ee_ref);

    vec12 p_hip = m_robot.getHFEPosition(js_ref);
    Eigen::Vector3d p_hip_i = p_hip.block<3, 1>(3 * leg_id, 0);

    Eigen::Vector3d ang_correction = base_w_des.cross(p_hip_i - m_joint_state_act.block<3, 1>(0, 0));

    float gravity = 9.81;
    p_step_i.block<2, 1>(0, 0) = p_hip_i.block<2, 1>(0, 0) + 0.5 * t_stance * base_ref_vel +
                                 0.7 * m_use_capture_point_heuristic * std::sqrt(p_hip_i(2) / gravity) * (0.5 * (base_vel_act + ang_correction.block<2, 1>(0, 0)) - base_ref_vel);

    return p_step_i;
}

void ReactivePlanner::SetBaseTarget() {
    float t = m_t_curr;

    if (use_vpsp)
    {
        Eigen::MatrixXd p_f_VPSP = GetVPSPVertices(m_t_curr);
        Eigen::Vector2d base_pos_from_VPSP = p_f_VPSP.rowwise().mean();
        m_ee_state_ref(0) = base_pos_from_VPSP(0);
        m_ee_state_ref(1) = base_pos_from_VPSP(1);

        m_ee_vel_ref(0) = m_v_cmd_x;
        m_ee_vel_ref(1) = m_v_cmd_y;

    }
    else
    {
        m_ee_vel_ref(0) = m_v_cmd_x;
        m_ee_vel_ref(1) = m_v_cmd_y;

        m_ee_state_ref(0) = m_ee_state_ref(0) + m_v_cmd_x * m_dt;
        m_ee_state_ref(1) = m_ee_state_ref(1) + m_v_cmd_y * m_dt;

    }

    m_ee_acc_ref(5) = 0;
    m_ee_vel_ref(5) = m_v_cmd_yaw;

    // float yaw = fmod(m_ee_ref.position[5] + m_v_cmd_yaw * m_dt, 2 * M_PI);

    // if (yaw > M_PI)
    // {
    //    m_ee_ref.position[5] = yaw - 2 * M_PI;
    // }
    // else if (yaw < -M_PI)
    // {
    //    m_ee_ref.position[5] = yaw + 2 * M_PI;
    // }
    // else
    // {
    //    m_ee_ref.position[5] = yaw;
    // }
    // m_margin.data = GetVPSPStabilityMargin();
    // std::cout << "margin: " << m_margin.data << "\n";
    // m_margin_pub.publish(m_margin);
    // m_polygon_pub.publish(m_polygon);
}

bool isOriginInsidePolygon(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
    // Ensure the input vectors have four elements
    if (x.size() != 4 || y.size() != 4) {
        throw std::invalid_argument("Input vectors must have four elements each.");
    }

    // Calculate cross products
    double AB = (x(1) - x(0)) * (y(1) + y(0));
    double BC = (x(2) - x(1)) * (y(2) + y(1));
    double CD = (x(3) - x(2)) * (y(3) + y(2));
    double DA = (x(0) - x(3)) * (y(0) + y(3));

    // Check if the origin is inside the polygon
    return (AB * BC * CD * DA > 0);
}

double distanceToPolygon(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
    // Ensure the input vectors have four elements
    if (x.size() != 4 || y.size() != 4) {
        throw std::invalid_argument("Input vectors must have four elements each.");
    }

    // Initialize minimum distance to a large value
    double minDistance = std::numeric_limits<double>::infinity();

    // Iterate over the edges of the polygon
    for (int i = 0; i < 4; ++i) {
        int next = (i + 1) % 4;

        // Calculate the vector representing the current edge
        Eigen::Vector2d edge(x(next) - x(i), y(next) - y(i));

        // Calculate the vector from the origin to a point on the edge
        Eigen::Vector2d originToEdge(x(i), y(i));

        // Calculate the perpendicular distance from the origin to the edge
        double distance = (originToEdge - edge.normalized() * originToEdge.dot(edge.normalized())).norm();

        // Update the minimum distance if needed
        minDistance = std::min(minDistance, distance);
    }
    
    bool inside = isOriginInsidePolygon(x, y);

    // if (inside) {
    //     return minDistance;
    // } else {
    //     return -minDistance;
    // }

    return minDistance;
}

double ReactivePlanner::GetVPSPStabilityMargin() {
    Eigen::Vector4d wt_VPSP = m_Pc_act;

    vec19 dummy_js = vec19::Zero();
    dummy_js.block<12,1>(7, 0) = m_theta;
    dummy_js.block<3,1>(3, 0) = m_joint_state_act.block<3,1>(3, 0);
    vec12 feet_pos = m_robot.forwardKinematics(dummy_js);
    // XY coordinates of the virtual predicitve support polygon for each of the four legs
    Eigen::MatrixXd p_f_VPSP = Eigen::MatrixXd::Zero(2, 4);

    // an array for storing the adjacent legs
    Eigen::ArrayXi adj_leg = Eigen::ArrayXi::Zero(6);
    adj_leg << 3, 0, 1, 2, 3, 0;

    for (int j = 1; j < 5; j++) {
        int i = j - 1;
        int i_prev = adj_leg(j - 1);
        int i_next = adj_leg(j + 1);

        Eigen::Vector2d p_i, p_i_prev, p_i_next;
        p_i << feet_pos(3 * i), feet_pos(3 * i + 1);
        p_i_prev << feet_pos(3 * i_prev), feet_pos(3 * i_prev + 1);
        p_i_next << feet_pos(3 * i_next), feet_pos(3 * i_next + 1);

        float phi_i, phi_i_prev, phi_i_next;
        phi_i = wt_VPSP(i);
        phi_i_prev = wt_VPSP(i_prev);
        phi_i_next = wt_VPSP(i_next);

        Eigen::Vector2d p_f_VPSP_prev, p_f_VPSP_next;

        p_f_VPSP_prev = phi_i * p_i + (1 - phi_i) * p_i_prev;
        p_f_VPSP_next = phi_i * p_i + (1 - phi_i) * p_i_next;

        p_f_VPSP.block<2, 1>(0, i) = (phi_i * p_i + phi_i_prev * p_f_VPSP_prev + phi_i_next * p_f_VPSP_next) / (phi_i + phi_i_prev + phi_i_next);
        // m_polygon.data[2 * i] = p_f_VPSP(0, i);
        // m_polygon.data[2 * i + 1] = p_f_VPSP(1, i);
    }

    float margin = distanceToPolygon(p_f_VPSP.block<1,4>(0, 0), p_f_VPSP.block<1,4>(1, 0));

    return margin;
}

Eigen::MatrixXd ReactivePlanner::GetVPSPVertices(const float &t) {
    Eigen::Vector4d wt_VPSP = getScheduledContactProbability(t, m_gait);

    // XY coordinates of the virtual predicitve support polygon for each of the four legs
    Eigen::MatrixXd p_f_VPSP = Eigen::MatrixXd::Zero(2, 4);

    // an array for storing the adjacent legs
    Eigen::ArrayXi adj_leg = Eigen::ArrayXi::Zero(6);
    adj_leg << 3, 0, 1, 2, 3, 0;

    for (int j = 1; j < 5; j++)
    {
        int i = j - 1;
        int i_prev = adj_leg(j - 1);
        int i_next = adj_leg(j + 1);

        Eigen::Vector2d p_i, p_i_prev, p_i_next;
        p_i << m_ee_state_ref(7 + 3 * i), m_ee_state_ref(7 + 3 * i + 1);
        p_i_prev << m_ee_state_ref(7 + 3 * i_prev), m_ee_state_ref(7 + 3 * i_prev + 1);
        p_i_next << m_ee_state_ref(7 + 3 * i_next), m_ee_state_ref(7 + 3 * i_next + 1);

        float phi_i, phi_i_prev, phi_i_next;
        phi_i = wt_VPSP(i);
        phi_i_prev = wt_VPSP(i_prev);
        phi_i_next = wt_VPSP(i_next);

        Eigen::Vector2d p_f_VPSP_prev, p_f_VPSP_next;

        p_f_VPSP_prev = phi_i * p_i + (1 - phi_i) * p_i_prev;
        p_f_VPSP_next = phi_i * p_i + (1 - phi_i) * p_i_next;

        p_f_VPSP.block<2, 1>(0, i) = (phi_i * p_i + phi_i_prev * p_f_VPSP_prev + phi_i_next * p_f_VPSP_next) / (phi_i + phi_i_prev + phi_i_next);
    }

    return p_f_VPSP;
}