#pragma once

#include "CommunicationManager.hpp"

#include <functional>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
// using std::placeholders::_1;

class QuadROSComm : public rclcpp::Node, public CommunicationManager {
public:
    QuadROSComm();
    QuadROSComm(const std::string& name);
    ~QuadROSComm();

    void Run();
private:
    void InitClass();
    std::string m_name;
    float m_loop_rate = 1000;
    float m_dt = 0.001;
    // Subscriber(s)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    // Publisher(s)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;

    // Timer
    rclcpp::TimerBase::SharedPtr m_timer;

    // Subscription callbacks
    void get_joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg) const;
    void get_imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) const;
    void get_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) const;
    // Timer callback
    void timer_cb();

    sensor_msgs::msg::JointState js_data;
};

class CommROSP2E : public rclcpp::Node, public CommunicationManager {
public:
    CommROSP2E();
    CommROSP2E(const std::string& name);
    ~CommROSP2E();

    void Run();
private:
    void InitClass();
    std::string m_name;
    float m_loop_rate = 1000;
    float m_dt = 0.001;
    // Subscriber(s)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_cmd_sub;
    // Publisher(s)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    // Timer
    rclcpp::TimerBase::SharedPtr m_timer;

    // Subscription callbacks
    void get_joint_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg) const;
    // Timer callback
    void timer_cb();

    sensor_msgs::msg::JointState js_data;
    sensor_msgs::msg::Imu imu_data;
    nav_msgs::msg::Odometry odom_data;
};