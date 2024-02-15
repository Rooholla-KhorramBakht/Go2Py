#include "QuadROSComm.hpp"

QuadROSComm::QuadROSComm() : m_name("svan"), rclcpp::Node("svan"), CommunicationManager(DATA_ACCESS_MODE::EXECUTOR_TO_PLANT) {
    InitClass();
}

QuadROSComm::QuadROSComm(const std::string& name) : m_name(name), rclcpp::Node(name), CommunicationManager(name, DATA_ACCESS_MODE::EXECUTOR_TO_PLANT) {
    InitClass();
}

QuadROSComm::~QuadROSComm() {
    rclcpp::shutdown();
}

void QuadROSComm::InitClass() {
    m_dt = 1./m_loop_rate;

    m_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/" + m_name + "/joint_cmd", 1);
    auto pub_timer = std::chrono::duration<double>(m_dt);
    m_timer = this->create_wall_timer(pub_timer, std::bind(&QuadROSComm::timer_cb, this));
    
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/" + m_name + "/joint_state", 
        1, 
        std::bind(&QuadROSComm::get_joint_state_cb, this, std::placeholders::_1)
    );
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "/" + m_name + "/imu",
        1,
        std::bind(&QuadROSComm::get_imu_cb, this, std::placeholders::_1)
    );
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + m_name + "/odom",
        1,
        std::bind(&QuadROSComm::get_odom_cb, this, std::placeholders::_1)
    );

    js_data = sensor_msgs::msg::JointState();
    js_data.position.clear();
    js_data.velocity.clear();
    js_data.effort.clear();
    for (int i = 0; i < 12; ++i) {
        js_data.position.push_back(0);
        js_data.velocity.push_back(0);
        js_data.effort.push_back(0);
    }
}

void QuadROSComm::get_joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg) const {
    for (int i = 0; i < 12; ++i) {
        m_sensor_data_ptr -> q(i) = msg -> position[i];
        m_sensor_data_ptr -> qd(i) = msg -> velocity[i];
        m_sensor_data_ptr -> tau(i) = msg -> effort[i];
    }
}

void QuadROSComm::get_imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) const {
    m_sensor_data_ptr -> quat(0) = msg->orientation.x;
    m_sensor_data_ptr -> quat(1) = msg->orientation.y;
    m_sensor_data_ptr -> quat(2) = msg->orientation.z;
    m_sensor_data_ptr -> quat(3) = msg->orientation.w;

    m_sensor_data_ptr -> w_B(0) = msg->angular_velocity.x;
    m_sensor_data_ptr -> w_B(1) = msg->angular_velocity.y;
    m_sensor_data_ptr -> w_B(2) = msg->angular_velocity.z;
    
    m_sensor_data_ptr -> a_B(0) = msg->linear_acceleration.x;
    m_sensor_data_ptr -> a_B(1) = msg->linear_acceleration.y;
    m_sensor_data_ptr -> a_B(2) = msg->linear_acceleration.z;

    // TODO: populate a_W and w_W as well
}

void QuadROSComm::get_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    // TODO: Use Odom data as required. Maybe extend sensor_data to have odom data?
}

void QuadROSComm::timer_cb() {
    for (int i = 0; i < 12; ++i) {
        js_data.position[i] = m_joint_command_data_ptr -> q(i);
        js_data.velocity[i] = m_joint_command_data_ptr -> qd(i);
        js_data.effort[i] = m_joint_command_data_ptr -> tau(i);
    }
    m_joint_state_pub->publish(js_data);
}

void QuadROSComm::Run() {
    // rclcpp::init(0, nullptr);
    rclcpp::spin(shared_from_this());
}

/***********************************************************/

CommROSP2E::CommROSP2E() : m_name("svan"), rclcpp::Node("svan"), CommunicationManager(DATA_ACCESS_MODE::PLANT_TO_EXECUTOR) {
    InitClass();
}

CommROSP2E::CommROSP2E(const std::string& name) : m_name(name), rclcpp::Node(name), CommunicationManager(name, DATA_ACCESS_MODE::PLANT_TO_EXECUTOR) {
    InitClass();
}

CommROSP2E::~CommROSP2E() {
    rclcpp::shutdown();
}

void CommROSP2E::InitClass() {
    m_dt = 1./m_loop_rate;

    m_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/" + m_name + "/joint_state", 1);
    m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/" + m_name + "/imu", 1);
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/" + m_name + "/odom", 1);

    auto pub_timer = std::chrono::duration<double>(m_dt);
    m_timer = this->create_wall_timer(pub_timer, std::bind(&CommROSP2E::timer_cb, this));
    
    m_joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/" + m_name + "/joint_cmd", 
        1, 
        std::bind(&CommROSP2E::get_joint_cmd_cb, this, std::placeholders::_1)
    );

    js_data = sensor_msgs::msg::JointState();
    imu_data = sensor_msgs::msg::Imu();
    odom_data = nav_msgs::msg::Odometry();

    js_data.position.clear();
    js_data.velocity.clear();
    js_data.effort.clear();
    for (int i = 0; i < 12; ++i) {
        js_data.position.push_back(0);
        js_data.velocity.push_back(0);
        js_data.effort.push_back(0);
    }
}

void CommROSP2E::get_joint_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg) const {
    for (int i = 0; i < 12; ++i) {
        m_joint_command_data_ptr -> q(i) = msg -> position[i];
        m_joint_command_data_ptr -> qd(i) = msg -> velocity[i];
        m_joint_command_data_ptr -> tau(i) = msg -> effort[i];
    }
}

void CommROSP2E::timer_cb() {
    for (int i = 0; i < 12; ++i) {
        js_data.position[i] = m_sensor_data_ptr -> q(i);
        js_data.velocity[i] = m_sensor_data_ptr -> qd(i);
        js_data.effort[i] = m_sensor_data_ptr -> tau(i);
    }
    // Populate imu_data
    imu_data.orientation.w = m_sensor_data_ptr -> quat(3);
    imu_data.orientation.x = m_sensor_data_ptr -> quat(0);
    imu_data.orientation.y = m_sensor_data_ptr -> quat(1);
    imu_data.orientation.z = m_sensor_data_ptr -> quat(2);
    imu_data.linear_acceleration.x = m_sensor_data_ptr -> a_B(0);
    imu_data.linear_acceleration.y = m_sensor_data_ptr -> a_B(1);
    imu_data.linear_acceleration.z = m_sensor_data_ptr -> a_B(2);
    imu_data.angular_velocity.x = m_sensor_data_ptr -> w_B(0);
    imu_data.angular_velocity.y = m_sensor_data_ptr -> w_B(1);
    imu_data.angular_velocity.z = m_sensor_data_ptr -> w_B(2);
    // Populdate odometry data

    // Publish stuff
    m_joint_state_pub->publish(js_data);
    m_imu_pub->publish(imu_data);
    m_odom_pub->publish(odom_data);
}

void CommROSP2E::Run() {
    // rclcpp::init(0, nullptr);
    rclcpp::spin(shared_from_this());
}