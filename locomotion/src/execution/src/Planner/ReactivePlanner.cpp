#include "Planner/ReactivePlanner.hpp"
#include <cmath>

ReactivePlanner::ReactivePlanner() : Planner("svan"), m_name("svan_planner")
{
    initClass();
}

ReactivePlanner::ReactivePlanner(std::string name) : Planner(name), m_name(name)
{
    initClass();
}

void ReactivePlanner::initClass() {
}

vec3 ReactivePlanner::getDesiredFootPosition(const uint8_t &leg_id, const float &t_stance) {
    vec3 p_step_i = vec3::Zero();

    Eigen::Vector2d p_vel_ref = m_ee_vel_ref.block<2,1>(0, 0);
    Eigen::Vector2d base_vel_ref = m_ee_vel_ref.block<2,1>(0, 0);
    Eigen::Vector2d base_vel_act = m_joint_vel_act.block<2, 1>(0, 0);

    vec3 base_w_des = m_ee_vel_ref.block<3,1>(3, 0);
    vec3 base_w_act = m_joint_vel_act.block<3,1>(3, 0);

    vec3 base_pos_ref = m_ee_state_ref.block<3,1>(0, 0);

    vec19 js_ref = vec19::Zero();
    js_ref.block<7,1>(0, 0) = m_ee_state_ref.block<7,1>(0, 0);
    js_ref.block<12,1>(7, 0) = m_robot.inverseKinematics(m_ee_state_ref);

    Eigen::Vector3d p_hip_i = m_robot.getHFEPosition(js_ref).block<3, 1>(3 * leg_id, 0);

    vec3 r_hip_B = p_hip_i - base_pos_ref;
    vec3 ang_correction_ref = base_w_des.cross(r_hip_B);
    vec3 ang_correction_act = base_w_act.cross(r_hip_B);
    
    Eigen::Vector2d v_hip_ref = base_vel_ref + ang_correction_ref.block<2,1>(0, 0);
    Eigen::Vector2d v_hip_act = base_vel_act + ang_correction_act.block<2,1>(0, 0);

    float gravity = 9.81;

    Eigen::Vector2d cp_cmd = m_use_capture_point_heuristic * std::sqrt(p_hip_i(2) / gravity) * (v_hip_act - v_hip_ref);
    // for (int i = 0; i < 2; ++i) {
    //     float delta_v = std::fabs(v_hip_act(i)) - std::fabs(v_hip_ref(i));
    //     if (delta_v < 0) {
    //         cp_cmd(i) = 0;
    //     }
    // }

    // std::cout << "Raibert heiristic: " << leg_id << ": " << (getRaibertHeuristic(leg_id, t_stance) - p_hip_i).block<2,1>(0, 0).transpose() << "\n";
    // std::cout << "Capture point command: " << leg_id << ": " << cp_cmd.transpose() << "\n";
    // std::cout << "Base vel ref: " << base_vel_ref.transpose() << "\n";

    p_step_i.block<2, 1>(0, 0) = getRaibertHeuristic(leg_id, t_stance).block<2,1>(0, 0) + 0.0 * cp_cmd;

    return p_step_i;
}

void ReactivePlanner::setTarget() {
    setFeetTarget();
    setBaseTarget();

    if (use_vpsp) {
        Eigen::MatrixXd p_f_VPSP = getVPSPVertices(m_t_curr);
        // std::cout << "polygon vertices: \n" << p_f_VPSP << "\n";
        Eigen::Vector2d base_pos_from_VPSP = p_f_VPSP.rowwise().mean();
        m_ee_state_ref.block<2,1>(0, 0) = base_pos_from_VPSP;
    }

    // std::cout << "Base velocity: " << m_ee_vel_ref.block<2,1>(0, 0).transpose() << "\n";
    // std::cout << "Base acceleration: " << m_ee_acc_ref.block<2,1>(0, 0).transpose() << "\n";

    // std::cout << "Stability margin: " << getVPSPStabilityMargin() << "\n";
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

double ReactivePlanner::getVPSPStabilityMargin() {
    Eigen::Vector4d wt_VPSP = m_Pc_act;

    vec19 dummy_js = vec19::Zero();
    dummy_js.block<12,1>(7, 0) = m_theta;
    dummy_js.block<4,1>(3, 0) = m_joint_state_act.block<4,1>(3, 0);
    vec12 feet_pos = m_robot.forwardKinematics(dummy_js);
    // XY coordinates of the virtual predicitve support polygon for each of the four legs
    Eigen::MatrixXd p_f_VPSP = Eigen::MatrixXd::Zero(2, 4);

    // an array for storing the adjacent legs
    Eigen::ArrayXi adj_leg = Eigen::ArrayXi::Zero(6);
    // adj_leg << 3, 0, 1, 2, 3, 0;
    adj_leg << 3, 1, 0, 2, 3, 1;

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

    // p_f_VPSP = getVPSPVertices();

    return distanceToPolygon(p_f_VPSP.block<1,4>(0, 0), p_f_VPSP.block<1,4>(1, 0));
}

Eigen::MatrixXd ReactivePlanner::getVPSPVertices(const float &t) {
    Eigen::Vector4d wt_VPSP = getScheduledContactProbability(t, m_gait);

    // XY coordinates of the virtual predicitve support polygon for each of the four legs
    Eigen::MatrixXd p_f_VPSP = Eigen::MatrixXd::Zero(2, 4);

    // an array for storing the adjacent legs
    // Eigen::ArrayXi adj_leg = Eigen::ArrayXi::Zero(6);
    Eigen::MatrixXi adj_leg = Eigen::MatrixXi::Zero(4, 2);
    // adj_leg << 3, 2, 0, 1, 3, 2;
    // adj_leg << 3, 0, 1, 2, 3, 0;
    // adj_leg << 3, 0, 1, 2, 3, 0;
    adj_leg << 1, 2,
                3, 0,
                0, 3,
                2, 1;

    for (int j = 1; j < 5; j++)
    {
        int i = j - 1;
        // int i_prev = adj_leg(j - 1);
        // int i_next = adj_leg(j + 1);
        int i_prev = adj_leg(i, 0);
        int i_next = adj_leg(i, 1);

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