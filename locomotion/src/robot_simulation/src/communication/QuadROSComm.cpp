#include "QuadROSComm.hpp"

QuadROSComm::QuadROSComm(const DATA_ACCESS_MODE& mode) : m_name("svan"), rclcpp::Node("svan"), CommunicationManager(mode) {
    InitClass();
}

QuadROSComm::QuadROSComm(const std::string& name, const DATA_ACCESS_MODE& mode) : m_name(name), rclcpp::Node(name), CommunicationManager(name, mode) {
    InitClass();
}

QuadROSComm::~QuadROSComm() {
    rclcpp::shutdown();
}

void QuadROSComm::InitClass() {
    m_dt = 1./m_loop_rate;

    auto pub_timer = std::chrono::duration<double>(m_dt);
    m_timer = this->create_wall_timer(pub_timer, std::bind(&QuadROSComm::timer_cb, this));

    if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        m_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/" + m_name + "/joint_cmd", 1);
        
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
        for (int i = 0; i < 4; ++i) {
            m_est_cs_pub[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/" + m_name + "/estimation/contact_state/cs" + std::to_string(i),
                1
            );
            m_est_pc_pub[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/" + m_name + "/estimation/contact_probability/pc" + std::to_string(i),
                1
            );
            est_cs_data[i] = std_msgs::msg::Float32();
            est_pc_data[i] = std_msgs::msg::Float32();
            est_cs_data[i].data = 0;
            est_pc_data[i].data = 0;
        }
        for (int i = 0; i < 3; ++i) {
            m_est_base_position_pub[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/" + m_name + "/estimation/base_position/rB" + std::to_string(i),
                1
            );
            m_est_base_velocity_pub[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/" + m_name + "/estimation/base_velocity/vB" + std::to_string(i),
                1
            );
            est_base_pos_data[i] = std_msgs::msg::Float32();
            est_base_vel_data[i] = std_msgs::msg::Float32();
            est_base_pos_data[i].data = 0;
            est_base_vel_data[i].data = 0;
        }
        for (int i = 0; i < 12; ++i) {
            // m_contact_forces_pub[i] = this->create_publisher<std_msgs::msg::Float32>("/" + m_name + "/sim/contact_forces_" + std::to_string(i), 1);
            m_estimated_fc_pub[i] = this->create_publisher<std_msgs::msg::Float32>("/" + m_name + "/estimation/contact_forces_" + std::to_string(i), 1);
        }
    } else if (m_mode == DATA_ACCESS_MODE::PLANT) {
        m_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/" + m_name + "/joint_state", 1);
        m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/" + m_name + "/imu", 1);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/" + m_name + "/odom", 1);
        m_base_position_pub = this->create_publisher<geometry_msgs::msg::Point>("/" + m_name + "/sim/base_position", 1);
        m_base_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/" + m_name + "/sim/base_velocity", 1);
        m_base_acc_pub = this->create_publisher<geometry_msgs::msg::Twist>("/" + m_name + "/sim/base_acceleration", 1);
        for (int i = 0; i < 12; ++i) {
            m_contact_forces_pub[i] = this->create_publisher<std_msgs::msg::Float32>("/" + m_name + "/sim/contact_forces_" + std::to_string(i), 1);
            // m_estimated_fc_pub[i] = this->create_publisher<std_msgs::msg::Float32>("/" + m_name + "/estimation/contact_forces_" + std::to_string(i), 1);
        }

        // auto pub_timer = std::chrono::duration<double>(m_dt);
        // m_timer = this->create_wall_timer(pub_timer, std::bind(&QuadROSComm::timer_cb, this));
        
        m_joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/" + m_name + "/joint_cmd", 
            1, 
            std::bind(&QuadROSComm::get_joint_cmd_cb, this, std::placeholders::_1)
        );

        
    }

    js_data = sensor_msgs::msg::JointState();
    js_data.position.clear();
    js_data.velocity.clear();
    js_data.effort.clear();
    for (int i = 0; i < 12; ++i) {
        js_data.position.push_back(0);
        js_data.velocity.push_back(0);
        js_data.effort.push_back(0);
    }

    // js_data = sensor_msgs::msg::JointState();
    imu_data = sensor_msgs::msg::Imu();
    odom_data = nav_msgs::msg::Odometry();
    base_position_data = geometry_msgs::msg::Point();
    base_velocity_data = geometry_msgs::msg::Twist();
    base_acceleration_data = geometry_msgs::msg::Twist();
    for (int i = 0; i < 12; ++i) {
        contact_forces_data[i] = std_msgs::msg::Float32();
        contact_forces_data[i].data = 0;
        estimated_fc_data[i] = std_msgs::msg::Float32();
        estimated_fc_data[i].data = 0;
    }

    // js_data.position.clear();
    // js_data.velocity.clear();
    // js_data.effort.clear();
    // for (int i = 0; i < 12; ++i) {
    //     js_data.position.push_back(0);
    //     js_data.velocity.push_back(0);
    //     js_data.effort.push_back(0);
    // }
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

void QuadROSComm::get_joint_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg) const {
    for (int i = 0; i < 12; ++i) {
        m_joint_command_data_ptr -> q(i) = msg -> position[i];
        m_joint_command_data_ptr -> qd(i) = msg -> velocity[i];
        m_joint_command_data_ptr -> tau(i) = msg -> effort[i];
    }
}

void QuadROSComm::timer_cb() {
    if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        for (int i = 0; i < 12; ++i) {
            js_data.position[i] = m_joint_command_data_ptr -> q(i);
            js_data.velocity[i] = m_joint_command_data_ptr -> qd(i);
            js_data.effort[i] = m_joint_command_data_ptr -> tau(i);
        }
        for (int i = 0; i < 4; ++i) {
            est_cs_data[i].data = m_estimation_data_ptr -> cs(i);
            est_pc_data[i].data = m_estimation_data_ptr -> pc(i);
            m_est_cs_pub[i]->publish(est_cs_data[i]);
            m_est_pc_pub[i]->publish(est_pc_data[i]);
        }
        for (int i = 0; i < 3; ++i) {
            est_base_pos_data[i].data = m_estimation_data_ptr -> rB(i);
            est_base_vel_data[i].data = m_estimation_data_ptr -> vB(i);
            m_est_base_position_pub[i]->publish(est_base_pos_data[i]);
            m_est_base_velocity_pub[i]->publish(est_base_vel_data[i]);
        }

        for (int i = 0; i < 12; ++i) {
            estimated_fc_data[i].data = m_measurement_data_ptr -> estimated_contact_force(i);
            m_estimated_fc_pub[i]->publish(estimated_fc_data[i]);
        }

        m_joint_state_pub->publish(js_data);
    } else if (m_mode == DATA_ACCESS_MODE::PLANT) {
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

        // Populate measurement data from sim
        base_position_data.x = m_measurement_data_ptr -> base_position(0);
        base_position_data.y = m_measurement_data_ptr -> base_position(1);
        base_position_data.z = m_measurement_data_ptr -> base_position(2);

        base_velocity_data.linear.x = m_measurement_data_ptr -> base_velocity.linear(0);
        base_velocity_data.linear.y = m_measurement_data_ptr -> base_velocity.linear(1);
        base_velocity_data.linear.z = m_measurement_data_ptr -> base_velocity.linear(2);
        base_velocity_data.angular.x = m_measurement_data_ptr -> base_velocity.angular(0);
        base_velocity_data.angular.y = m_measurement_data_ptr -> base_velocity.angular(1);
        base_velocity_data.angular.z = m_measurement_data_ptr -> base_velocity.angular(2);

        base_acceleration_data.linear.x = m_measurement_data_ptr -> base_acceleration.linear(0);
        base_acceleration_data.linear.y = m_measurement_data_ptr -> base_acceleration.linear(1);
        base_acceleration_data.linear.z = m_measurement_data_ptr -> base_acceleration.linear(2);
        base_acceleration_data.angular.x = m_measurement_data_ptr -> base_acceleration.angular(0);
        base_acceleration_data.angular.y = m_measurement_data_ptr -> base_acceleration.angular(1);
        base_acceleration_data.angular.z = m_measurement_data_ptr -> base_acceleration.angular(2);

        for (int i = 0; i < 12; ++i) {
            contact_forces_data[i].data = m_measurement_data_ptr -> contact_force(i);
            m_contact_forces_pub[i]->publish(contact_forces_data[i]);
            // estimated_fc_data[i].data = m_measurement_data_ptr -> estimated_contact_force(i);
            // m_estimated_fc_pub[i]->publish(estimated_fc_data[i]);
        }

        // Publish stuff
        m_joint_state_pub->publish(js_data);
        m_imu_pub->publish(imu_data);
        m_odom_pub->publish(odom_data);
        m_base_position_pub->publish(base_position_data);
        m_base_vel_pub->publish(base_velocity_data);
        m_base_acc_pub->publish(base_acceleration_data);
    }
}

void QuadROSComm::Run() {
    rclcpp::spin(shared_from_this());
}