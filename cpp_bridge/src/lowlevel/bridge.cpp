#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <go2py/LowCmd.hpp>
#include <go2py/LowState.hpp>
#include <thread>
#include "utils/dds_subscriber.hpp"
#include "utils/dds_publisher.hpp"


using namespace unitree::common;
using namespace unitree::robot;
using namespace org::eclipse::cyclonedds;
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

class Go2LowLevelBridge
{
public:
    explicit Go2LowLevelBridge(const std::string &input_cmd_topic_name, 
                    const std::string &output_state_topic_name,
                    const int domain_id = 0):
        lowstate_publisher(output_state_topic_name), 
        lowcmd_subscriber(input_cmd_topic_name, std::bind(&Go2LowLevelBridge::go2py_callback, this, std::placeholders::_1))  
        {
            ChannelFactory::Instance()->Init(domain_id);
        }

    ~Go2LowLevelBridge()
    {
    }
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
    std::thread go2py_thread;
    bool running = true;
    void go2py_callback(const msgs::LowCmd& msg);
    DDSPublisher<msgs::LowState> lowstate_publisher;
    DDSSubscriber<msgs::LowCmd> lowcmd_subscriber;


private:
    float dt = 0.002; // 0.001~0.01
    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init
    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
};

void Go2LowLevelBridge::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();
    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Go2LowLevelBridge::LowStateMessageHandler, this, std::placeholders::_1), 1);
    /*loop publishing thread*/
    // lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Go2LowLevelBridge::LowCmdWrite, this);  
}

void Go2LowLevelBridge::go2py_callback(const msgs::LowCmd& msg)
{
    for(int i=0; i<12; i++)
    {
        low_cmd.motor_cmd()[i].q() = msg.q()[i];
        low_cmd.motor_cmd()[i].dq() = msg.dq()[i];
        low_cmd.motor_cmd()[i].kp() = msg.kp()[i];
        low_cmd.motor_cmd()[i].kd() = msg.kv()[i];
        low_cmd.motor_cmd()[i].tau() = msg.tau_ff()[i];
    }
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    lowcmd_publisher->Write(low_cmd);
}

void Go2LowLevelBridge::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Go2LowLevelBridge::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
    msgs::LowState lowstate;
    
    for(int i=0; i<12; i++)
    {
        lowstate.q()[i] = low_state.motor_state()[i].q();
        lowstate.dq()[i] = low_state.motor_state()[i].dq();
        lowstate.ddq()[i] = low_state.motor_state()[i].ddq();
        lowstate.tau_est()[i] = low_state.motor_state()[i].tau_est();
        lowstate.tmp()[i] = (float)low_state.motor_state()[i].temperature();
    }

    lowstate.voltage() = (float)low_state.power_v();
    lowstate.current() = (float)low_state.power_a();
    
    for(int i=0; i<4; i++)
        lowstate.contact()[i] = (float)low_state.foot_force()[i];

    lowstate_publisher.publish(lowstate);
}

void Go2LowLevelBridge::LowCmdWrite()
{
    low_cmd.motor_cmd()[2].q() = 0;
    low_cmd.motor_cmd()[2].dq() = 0;
    low_cmd.motor_cmd()[2].kp() = 0;
    low_cmd.motor_cmd()[2].kd() = 0;
    low_cmd.motor_cmd()[2].tau() = 0;
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    // ChannelFactory::Instance()->Init(0, argv[1]);
    Go2LowLevelBridge bridge("go2py/lowcmd", "go2py/lowstate");
    bridge.Init();
    while (1)
    {
        sleep(10);
    }

    return 0;
}
