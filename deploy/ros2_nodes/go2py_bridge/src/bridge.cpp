#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/motor_state.hpp"
#include "go2py_messages/msg/go2py_status.hpp"
#include "go2py_messages/msg/go2py_low_cmd.hpp"
#include "go2py_messages/msg/go2py_high_cmd.hpp"
#include "go2py_messages/msg/go2py_state.hpp"

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "common/motor_crc.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>


// headers needed for highlevel control
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "joystick.h"
#include "unitree_api/msg/request.hpp"
class Custom: public rclcpp::Node
{
    public:
        Custom() : Node("go2py_bridge_node")
        {
            // Standard ROS2 Topics
            watchdog_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Custom::watchdog, this));
            pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/go2/odom", 1);
            pub_imu  = this->create_publisher<sensor_msgs::msg::Imu>("/go2/imu", 1);
            pub_joint  = this->create_publisher<sensor_msgs::msg::JointState>("/go2/joint_states", 1);
            nav2_twist_subr = this->create_subscription<geometry_msgs::msg::Twist>("/go2/cmd_vel", 1, std::bind(&Custom::nav2TwistCmdCallback, this, std::placeholders::_1));
            
            //Unitree Topics
            init_lowcmd();
            unitree_lowstate_suber = this->create_subscription<unitree_go::msg::LowState>(
            "lowstate", 1, std::bind(&Custom::unitree_lowstate_callback, this, std::placeholders::_1));
            utlidar_odom_subr = this->create_subscription<nav_msgs::msg::Odometry>(
            "/utlidar/robot_odom", 1, std::bind(&Custom::unitree_odom_callback, this, std::placeholders::_1));

            // the req_puber is set to subscribe "/api/sport/request" topic with dt
            unitree_highreq_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 1);
            lowcmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 1);
            
            // Go2py topics
            go2py_low_cmd_suber = this->create_subscription<go2py_messages::msg::Go2pyLowCmd>(
            "/go2py/low_cmd", 1, std::bind(&Custom::go2py_low_cmd_callback, this, std::placeholders::_1));
            go2py_high_cmd_suber = this->create_subscription<go2py_messages::msg::Go2pyHighCmd>(
            "/go2py/high_cmd", 1, std::bind(&Custom::go2py_high_cmd_callback, this, std::placeholders::_1));
            go2py_state_puber = this->create_publisher<go2py_messages::msg::Go2pyState>("/go2py/state", 1);
            status_publisher = this->create_publisher<go2py_messages::msg::Go2pyStatus>("/go2py/status", 1);
            api_publisher = this->create_publisher<unitree_api::msg::Request>("/api/robot_state/request", 1);
        }
    private:
        rclcpp::TimerBase::SharedPtr watchdog_timer;
        void watchdog();
        void init_lowcmd();
        int Estop = 0;
        int sport_mode = 1;
        xRockerBtnDataStruct _keyData;

        // Standard ROS2
        geometry_msgs::msg::TwistStamped twist_cmd;
        void nav2TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_twist_subr;        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr utlidar_odom_subr;        
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
        
        // Unitree Interface
        unitree_api::msg::Request highreq;
        unitree_go::msg::LowCmd lowcmd_msg;
        SportClient sport_req;
        void unitree_lowstate_callback(unitree_go::msg::LowState::SharedPtr data);
        void unitree_odom_callback(nav_msgs::msg::Odometry::SharedPtr data);
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr unitree_lowstate_suber;
        rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr unitree_highreq_puber;
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_puber;
        rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr api_publisher;

        // Lowlevel interface
        void go2py_low_cmd_callback(go2py_messages::msg::Go2pyLowCmd::SharedPtr data);
        void go2py_high_cmd_callback(go2py_messages::msg::Go2pyHighCmd::SharedPtr data);
        rclcpp::Subscription<go2py_messages::msg::Go2pyLowCmd>::SharedPtr go2py_low_cmd_suber;
        rclcpp::Subscription<go2py_messages::msg::Go2pyHighCmd>::SharedPtr go2py_high_cmd_suber;
        rclcpp::Publisher<go2py_messages::msg::Go2pyState>::SharedPtr go2py_state_puber;
        rclcpp::Publisher<go2py_messages::msg::Go2pyStatus>::SharedPtr status_publisher;  

};

void Custom::init_lowcmd()
{

    for (int i = 0; i < 20; i++)
    {
        lowcmd_msg.motor_cmd[i].mode = 0x01; //Set toque mode, 0x00 is passive mode
        lowcmd_msg.motor_cmd[i].q = PosStopF;
        lowcmd_msg.motor_cmd[i].kp = 0;
        lowcmd_msg.motor_cmd[i].dq = VelStopF;
        lowcmd_msg.motor_cmd[i].kd = 0;
        lowcmd_msg.motor_cmd[i].tau = 0;
    }
}

void Custom::unitree_odom_callback(nav_msgs::msg::Odometry::SharedPtr data)
{
    nav_msgs::msg::Odometry odom_state;
    odom_state.header.stamp = rclcpp::Clock().now();
    odom_state.header.frame_id = "odom";
    odom_state.child_frame_id = "base_link";
    odom_state.pose.pose.position.x = data->pose.pose.position.x;
    odom_state.pose.pose.position.y = data->pose.pose.position.y;
    odom_state.pose.pose.position.z = data->pose.pose.position.z;
    odom_state.pose.pose.orientation.x = data->pose.pose.orientation.x;
    odom_state.pose.pose.orientation.y = data->pose.pose.orientation.y;
    odom_state.pose.pose.orientation.z = data->pose.pose.orientation.z;
    odom_state.pose.pose.orientation.w = data->pose.pose.orientation.w;
    odom_state.twist.twist.linear.x = data->twist.twist.linear.x;
    odom_state.twist.twist.linear.y = data->twist.twist.linear.y;
    odom_state.twist.twist.linear.z = data->twist.twist.linear.z;
    odom_state.twist.twist.angular.x = data->twist.twist.angular.x;
    odom_state.twist.twist.angular.y = data->twist.twist.angular.y;
    odom_state.twist.twist.angular.z = data->twist.twist.angular.z;
    pub_odom->publish(odom_state);
}

void Custom::unitree_lowstate_callback(unitree_go::msg::LowState::SharedPtr data)
{
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::JointState joint_state;
    nav_msgs::msg::Odometry odom_state;
    go2py_messages::msg::Go2pyState go2py_state; 

    // Load the IMU message
    imu.header.stamp=rclcpp::Clock().now();
    imu.header.frame_id = "imu_link";
    imu.orientation.w = data->imu_state.quaternion[0];
    imu.orientation.x = data->imu_state.quaternion[1];
    imu.orientation.y = data->imu_state.quaternion[2];
    imu.orientation.z = data->imu_state.quaternion[3];
    imu.linear_acceleration.x = data->imu_state.accelerometer[0];
    imu.linear_acceleration.y = data->imu_state.accelerometer[1];
    imu.linear_acceleration.z = data->imu_state.accelerometer[2];
    imu.angular_velocity.x = data->imu_state.gyroscope[0];
    imu.angular_velocity.y = data->imu_state.gyroscope[1];
    imu.angular_velocity.z = data->imu_state.gyroscope[2];
    
    go2py_state.time_sec = imu.header.stamp.sec;
    go2py_state.time_nsec = imu.header.stamp.nanosec;
    go2py_state.quat[0] = imu.orientation.x;
    go2py_state.quat[1] = imu.orientation.y;
    go2py_state.quat[2] = imu.orientation.z;
    go2py_state.quat[3] = imu.orientation.w;
    for(int i=0; i<3; i++)
    {
        go2py_state.accel[i] = data->imu_state.accelerometer[i];
        go2py_state.gyro[i] = data->imu_state.gyroscope[i];
    }
    go2py_state.imu_temp = data->imu_state.temperature;

    // Load the joint state messages
    joint_state.header.stamp = imu.header.stamp;
    joint_state.header.frame_id = "trunk";
    joint_state.name.push_back("FR_hip_joint");
    joint_state.name.push_back("FR_thigh_joint");
    joint_state.name.push_back("FR_calf_joint");

    joint_state.name.push_back("FL_hip_joint");
    joint_state.name.push_back("FL_thigh_joint");
    joint_state.name.push_back("FL_calf_joint");

    joint_state.name.push_back("RR_hip_joint");
    joint_state.name.push_back("RR_thigh_joint");
    joint_state.name.push_back("RR_calf_joint");

    joint_state.name.push_back("RL_hip_joint");
    joint_state.name.push_back("RL_thigh_joint");
    joint_state.name.push_back("RL_calf_joint");

    for(int i=0; i<12; i++)
    {
        joint_state.position.push_back(data->motor_state[i].q);
        joint_state.velocity.push_back(data->motor_state[i].dq);
        joint_state.effort.push_back(data->motor_state[i].tau_est);
        go2py_state.q[i]=data->motor_state[i].q;
        go2py_state.dq[i]=data->motor_state[i].dq;
        go2py_state.tau[i]=data->motor_state[i].tau_est;
        go2py_state.motor_temp[i] = data->motor_state[i].temperature;
    }
    for(int i=0; i<4; i++)
        go2py_state.contact[i]=data->foot_force[i];

    for(int i=0; i<40; i++)
        go2py_state.wireless_remote[i]=data->wireless_remote[i];
    go2py_state.soc = data->bms_state.soc;
    pub_joint->publish(joint_state);
    pub_imu->publish(imu);
    go2py_state_puber->publish(go2py_state);
    // Check for emergency stop
    memcpy(&_keyData, &data->wireless_remote[0], 40);
    if (_keyData.btn.components.R2 == 1 && _keyData.btn.components.L2 == 1)
    {
        Estop = 1;
    }
    if ((_keyData.btn.components.L2 == 1 && _keyData.btn.components.A == 1))
    {
        Estop = 0;
    }
    if ((_keyData.btn.components.L2 == 1 && _keyData.btn.components.L1 == 1))
    {
        auto msg = std::make_shared<unitree_api::msg::Request>();
        // Populate the message fields
        msg->header.identity.id = 80005;
        msg->header.identity.api_id = 1001;
        msg->parameter = "{\"name\":\"sport_mode\",\"switch\":0}";
        api_publisher->publish(*msg);
        sport_mode = 0;
    }
    if ((_keyData.btn.components.R2 == 1 && _keyData.btn.components.R1 == 1))
    {
        auto msg = std::make_shared<unitree_api::msg::Request>();
        // Populate the message fields
        msg->header.identity.id = 80005;
        msg->header.identity.api_id = 1001;
        msg->parameter = "{\"name\":\"sport_mode\",\"switch\":1}";
        api_publisher->publish(*msg);
        sport_mode = 1;
    }
    // std::cout << "Estop: " << Estop << std::endl;
}

void Custom::nav2TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if(Estop == 0)
        sport_req.Move(highreq, msg->linear.x, msg->linear.y, msg->angular.z);
    else
        sport_req.Damp(highreq);
    
    // Publish request messages with desired body velocity
    unitree_highreq_puber->publish(highreq);
}

void Custom::go2py_high_cmd_callback(go2py_messages::msg::Go2pyHighCmd::SharedPtr data)
{
    if(Estop == 0)
        sport_req.Move(highreq, data->vx, data->vy, data->wz);
    else
        sport_req.Damp(highreq);
    
    // Publish request messages with desired body velocity
    unitree_highreq_puber->publish(highreq);   
}

void Custom::go2py_low_cmd_callback(go2py_messages::msg::Go2pyLowCmd::SharedPtr data)
{   
    if(Estop == 0)
    {
        for(int i=0; i<12; i++)
        {
            lowcmd_msg.motor_cmd[i].q =   data->q[i];   // Taregt angular(rad)
            lowcmd_msg.motor_cmd[i].kp =  data->kp[i];  // Poinstion(rad) control kp gain
            lowcmd_msg.motor_cmd[i].dq =  data->dq[i];  // Taregt angular velocity(rad/ss)
            lowcmd_msg.motor_cmd[i].kd =  data->kd[i];  // Poinstion(rad) control kd gain
            lowcmd_msg.motor_cmd[i].tau = data->tau[i]; // Feedforward toque 1N.m
            get_crc(lowcmd_msg); //Compute the CRC and load it into the message
        }
    }else
    {
        for(int i=0; i<12; i++)
        {
            lowcmd_msg.motor_cmd[i].q =   0.;   // Taregt angular(rad)
            lowcmd_msg.motor_cmd[i].kp =  0.;  // Poinstion(rad) control kp gain
            lowcmd_msg.motor_cmd[i].dq =  0.;  // Taregt angular velocity(rad/ss)
            lowcmd_msg.motor_cmd[i].kd =  0;  // Poinstion(rad) control kd gain
            lowcmd_msg.motor_cmd[i].tau = 0.; // Feedforward toque 1N.m
            get_crc(lowcmd_msg); //Compute the CRC and load it into the message
        }
    }
    lowcmd_puber->publish(lowcmd_msg); //Publish lowcmd message
}

void Custom::watchdog()
{
    if(Estop == 1)
    {
        sport_req.Damp(highreq);    
        unitree_highreq_puber->publish(highreq);
        for(int i=0; i<12; i++)
        {
            lowcmd_msg.motor_cmd[i].q =   0.;   // Taregt angular(rad)
            lowcmd_msg.motor_cmd[i].kp =  0.;  // Poinstion(rad) control kp gain
            lowcmd_msg.motor_cmd[i].dq =  0.;  // Taregt angular velocity(rad/ss)
            lowcmd_msg.motor_cmd[i].kd =  3;  // Poinstion(rad) control kd gain
            lowcmd_msg.motor_cmd[i].tau = 0.; // Feedforward toque 1N.m
        }
        get_crc(lowcmd_msg); //Compute the CRC and load it into the message
        lowcmd_puber->publish(lowcmd_msg);
    }
    auto msg = std::make_shared<go2py_messages::msg::Go2pyStatus>();
    msg->estop = Estop;
    msg->sport_mode = sport_mode;
    status_publisher->publish(*msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                          // Initialize rclcpp
    rclcpp::spin(std::make_shared<Custom>()); // Run ROS2 node which is make share with low_state_suber class
    rclcpp::shutdown();                                // Exit
    return 0;
}
