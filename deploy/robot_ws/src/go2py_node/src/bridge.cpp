#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/motor_state.hpp"
#include "unitree_go/msg/go2py_low_cmd.hpp"

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
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>


// headers needed for highlevel control
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "joystick.h"

class Custom: public rclcpp::Node
{
    public:
        Custom() : Node("go2py_bridge_node")
        {
            watchdog_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Custom::watchdog, this));
            pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/go2/odom", 1);
            pub_imu  = this->create_publisher<sensor_msgs::msg::Imu>("/go2/imu", 1);
            pub_joint  = this->create_publisher<sensor_msgs::msg::JointState>("/go2/joint_states", 1);
            sub_twist = this->create_subscription<geometry_msgs::msg::TwistStamped>("/go2/twist_cmd", 1, std::bind(&Custom::twistCmdCallback, this, std::placeholders::_1));
            
            // Go2 highlevel subscriber and publishers
            // the state_suber is set to subscribe "sportmodestate" topic
            highstate_suber = this->create_subscription<unitree_go::msg::SportModeState>(
                "sportmodestate", 10, std::bind(&Custom::highstate_callback, this, std::placeholders::_1));
            // the req_puber is set to subscribe "/api/sport/request" topic with dt
            highreq_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
            
            //Go2 lowlevel interface
            init_lowcmd();
            lowstate_suber = this->create_subscription<unitree_go::msg::LowState>(
            "lowstate", 1, std::bind(&Custom::lowstate_callback, this, std::placeholders::_1));
            
            lowcmd_suber = this->create_subscription<unitree_go::msg::Go2pyLowCmd>(
            "/go2/lowcmd", 1, std::bind(&Custom::lowcmd_callback, this, std::placeholders::_1));

            lowcmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

        }
    private:
        rclcpp::TimerBase::SharedPtr watchdog_timer;
        void watchdog();
        // Highlevel twist command through standard ROS2 message type
        void twistCmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        geometry_msgs::msg::TwistStamped twist_cmd;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist;
        uint64_t last_highcmd_stamp = 0;
        
        // ROS2 standard joint state, IMU, and odometry publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
        
        // Highlevel interface
        void highstate_callback(unitree_go::msg::SportModeState::SharedPtr data);
        rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr highstate_suber;
        rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr highreq_puber;
        unitree_api::msg::Request highreq; // Unitree Go2 ROS2 request message
        SportClient sport_req;
        
        // Lowlevel interface
        void lowstate_callback(unitree_go::msg::LowState::SharedPtr data);
        void lowcmd_callback(unitree_go::msg::Go2pyLowCmd::SharedPtr data);
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_suber;
        rclcpp::Subscription<unitree_go::msg::Go2pyLowCmd>::SharedPtr lowcmd_suber;
        // A struct to store the highlevel states for later use
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_puber;
        unitree_go::msg::LowCmd lowcmd_msg;
        uint64_t last_lowcmd_stamp = 0;
        xRockerBtnDataStruct _keyData;

        void init_lowcmd()
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

        struct {
            float px;
            float py;
            float height;
            float roll;
            float pitch;
            float yaw;
            float imu_temp;
            float wx;
            float wy;
            float wz; 
            float ax;
            float qy;
            float az; 
            float vx;
            float vy;
            float foot_force[4];
        }highstate;     
        int Estop = 0;
};

void Custom::lowstate_callback(unitree_go::msg::LowState::SharedPtr data)
{
    // std::cout << data->tick << std::endl;
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::JointState joint_state;
    nav_msgs::msg::Odometry odom_state;

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
    // add foot force joint state for contact detection
    joint_state.name.push_back("FR_foot_force");
    joint_state.name.push_back("FL_foot_force");
    joint_state.name.push_back("RR_foot_force");
    joint_state.name.push_back("RL_foot_force");

    for(int i=0; i<12; i++)
    {
        joint_state.position.push_back(data->motor_state[i].q);
        joint_state.velocity.push_back(data->motor_state[i].dq);
        joint_state.effort.push_back(data->motor_state[i].tau_est);
    }
    // use the last four joint_state.position to store the foot force
    for(int i=0; i<4; i++)
    {
        joint_state.effort.push_back(data->foot_force[i]);
    }
    pub_joint->publish(joint_state);
    pub_imu->publish(imu);
    memcpy(&_keyData, &data->wireless_remote[0], 40);
    if (_keyData.btn.components.R2 == 1 && _keyData.btn.components.L2 == 1)
    {
        Estop = 1;
    }
    if (_keyData.btn.components.R1 == 1 && _keyData.btn.components.L1 == 1)
    {
        Estop = 0;
    }
    // std::cout << "Estop: " << Estop << std::endl;
}

void Custom::highstate_callback(unitree_go::msg::SportModeState::SharedPtr data)
{
    highstate.px = data->position[0];
    highstate.py = data->position[1];
    highstate.roll = data->imu_state.rpy[0];
    highstate.pitch = data->imu_state.rpy[1];
    highstate.yaw = data->imu_state.rpy[2];
    for(int i=0; i<4; i++)
    {
        highstate.foot_force[i]=data->foot_force[i];
    }
    highstate.height = data->body_height;
    
    nav_msgs::msg::Odometry odom_state;
    // odometry states published by the onboard high-level controller
    odom_state.header.stamp = rclcpp::Clock().now();
    odom_state.header.frame_id = "odom";
    odom_state.child_frame_id = "base_link";
    odom_state.pose.pose.position.x = data->position[0];
    odom_state.pose.pose.position.y = data->position[1];
    odom_state.pose.pose.position.z = data->body_height;


    odom_state.twist.twist.linear.x = data->velocity[0];
    odom_state.twist.twist.linear.y = data->velocity[1];
    odom_state.twist.twist.linear.z = data->velocity[2];
    odom_state.twist.twist.angular.z= data->yaw_speed;
    double position_R = 0.05;
    double orientation_R = 0.005;
    odom_state.pose.covariance = {
        position_R, 0, 0, 0, 0, 0,  // Covariance for position x
        0, position_R, 0, 0, 0, 0,  // Covariance for position y
        0, 0, position_R, 0, 0, 0,  // Covariance for position z
        0, 0, 0, orientation_R, 0, 0,  // Covariance for orientation roll
        0, 0, 0, 0, orientation_R, 0,  // Covariance for orientation pitch
        0, 0, 0, 0, 0, orientation_R   // Covariance for orientation yaw    
    };

    odom_state.twist.covariance = {
        position_R, 0, 0, 0, 0, 0,  // Covariance for position x
        0, position_R, 0, 0, 0, 0,  // Covariance for position y
        0, 0, position_R, 0, 0, 0,  // Covariance for position z
        0, 0, 0, orientation_R, 0, 0,  // Covariance for orientation roll
        0, 0, 0, 0, orientation_R, 0,  // Covariance for orientation pitch
        0, 0, 0, 0, 0, orientation_R   // Covariance for orientation yaw    
    };
    
    pub_odom->publish(odom_state);
}

void Custom::twistCmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    auto stamp_now = std::chrono::high_resolution_clock::now();
    last_highcmd_stamp = std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();
    if(Estop == 0)
        sport_req.Move(highreq, msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);
    else
        sport_req.Damp(highreq);
    
    // Publish request messages with desired body velocity
    highreq_puber->publish(highreq);
}

void Custom::lowcmd_callback(unitree_go::msg::Go2pyLowCmd::SharedPtr data)
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
            lowcmd_puber->publish(lowcmd_msg); //Publish lowcmd message
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
            lowcmd_puber->publish(lowcmd_msg); //Publish lowcmd message
        }
    }
}

void Custom::watchdog()
{
    if(Estop == 1)
    {
        sport_req.Damp(highreq);    
        highreq_puber->publish(highreq);
        for(int i=0; i<12; i++)
        {
            lowcmd_msg.motor_cmd[i].q =   0.;   // Taregt angular(rad)
            lowcmd_msg.motor_cmd[i].kp =  0.;  // Poinstion(rad) control kp gain
            lowcmd_msg.motor_cmd[i].dq =  0.;  // Taregt angular velocity(rad/ss)
            lowcmd_msg.motor_cmd[i].kd =  3;  // Poinstion(rad) control kd gain
            lowcmd_msg.motor_cmd[i].tau = 0.; // Feedforward toque 1N.m
            get_crc(lowcmd_msg); //Compute the CRC and load it into the message
            lowcmd_puber->publish(lowcmd_msg); //Publish lowcmd message
        }
        lowcmd_puber->publish(lowcmd_msg);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                          // Initialize rclcpp
    rclcpp::spin(std::make_shared<Custom>()); // Run ROS2 node which is make share with low_state_suber class
    rclcpp::shutdown();                                // Exit
    return 0;
}