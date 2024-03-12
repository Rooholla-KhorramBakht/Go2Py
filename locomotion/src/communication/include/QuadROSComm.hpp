#pragma once

#include "CommunicationManager.hpp"

#include <functional>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
// using std::placeholders::_1;

class QuadROSComm : public rclcpp::Node, public CommunicationManager {
public:
    QuadROSComm(const DATA_ACCESS_MODE& mode);
    QuadROSComm(const std::string& name, const DATA_ACCESS_MODE& mode);
    ~QuadROSComm();

    void setUpdateRate(const float& rate) {
        m_loop_rate = rate;
        m_dt = 1./rate;
    }

    void run();
    void start_thread();

    void initClass();
private:
    std::string m_name;
    float m_loop_rate = 1000;
    float m_dt = 0.001;
    // Subscriber(s)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub;
    // Publisher(s)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_est_cs_pub[4];
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_est_pc_pub[4];
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_est_base_position_pub[3];
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_est_base_velocity_pub[3];

    // Subscriber(s)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_cmd_sub;
    // Publisher(s)
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    // Measurement publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_base_position_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_base_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_base_acc_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_contact_forces_pub[12];
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_estimated_fc_pub[12];

    // Timer
    rclcpp::TimerBase::SharedPtr m_timer;

    // Subscription callbacks
    void get_joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg) const;
    void get_imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) const;
    void get_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) const;
    // Subscription callbacks
    void get_joint_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg) const;

    void get_joystick_data_cb(const sensor_msgs::msg::Joy::SharedPtr msg) const;
    // Timer callback
    void timer_cb();

    // sensor_msgs::msg::JointState js_data;
    std_msgs::msg::Float32 est_cs_data[4];
    std_msgs::msg::Float32 est_pc_data[4];
    std_msgs::msg::Float32 est_base_pos_data[3];
    std_msgs::msg::Float32 est_base_vel_data[3];

    sensor_msgs::msg::JointState js_data;
    sensor_msgs::msg::Imu imu_data;
    nav_msgs::msg::Odometry odom_data;
    geometry_msgs::msg::Point base_position_data;
    geometry_msgs::msg::Twist base_velocity_data;
    geometry_msgs::msg::Twist base_acceleration_data;
    std_msgs::msg::Float32 contact_forces_data[12];
    std_msgs::msg::Float32 estimated_fc_data[12];

    sensor_msgs::msg::Joy joystick_data;

    std::thread m_thread;
};