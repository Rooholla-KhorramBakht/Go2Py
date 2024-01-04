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
#include "dds/dds.hpp"
#include "dds/dds.h"
#include <thread>


using namespace unitree::common;
using namespace unitree::robot;
using namespace org::eclipse::cyclonedds;
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);
class Bridge
{
public:
    explicit Bridge()
    {
    }

    ~Bridge()
    {
    }
    // participant = dds::domain::DomainParticipant(domain_id);
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
    std::thread go2py_thread;
    bool running = true;
    void go2py_callback();
    // Go2PyListener go2py_listener;
    // dds::sub::qos::DataReaderQos rqos;


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
    // dds::topic::Topic<unitree_go::msg::dds_::LowCmd_> topic(participant, "go2py/lowcmd");
};

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

void Bridge::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();
    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Bridge::LowStateMessageHandler, this, std::placeholders::_1), 1);
    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Bridge::LowCmdWrite, this);  
    go2py_thread = std::thread(&Bridge::go2py_callback, this);
}

void Bridge::go2py_callback()
{
    // dds::domain::DomainParticipant participant(domain::default_id());
    // /* To subscribe to something, a topic is needed. */
    // dds::topic::Topic<msgs::LowCmd> topic(participant, "go2py/lowcmd");
    // /* A reader also needs a subscriber. */
    // // dds::sub::Subscriber subscriber(participant);
    // dds::sub::NoOpSubscriberListener qlistener; /*you need to create your own class that derives from this listener, and implement your own callbacks*/
    // /*the listener implementation should implement the on_subscription_matched virtual function as we will rely on it later*/
    // dds::sub::qos::SubscriberQos subqos; /*add custom QoS policies that you want for this subscriber*/
    // dds::sub::Subscriber sub(participant, subqos, &qlistener, dds::core::status::StatusMask::subscription_matched());
    // /* Now, the reader can be created to subscribe to a HelloWorld message. */
    // dds::sub::NoOpAnyDataReaderListener listener; /*you need to create your own class that derives from this listener, and implement your own callback functions*/
    // /*the listener implementation should implement the on_data_available virtual function as we will rely on it later*/
    // dds::sub::qos::DataReaderQos rqos;
    // // dds::sub::DataReader<msgs::LowCmd> reader(subscriber, topic, rqos, &listener, dds::core::status::StatusMask::data_available());
    // dds::sub::DataReader<msgs::LowCmd> reader(sub, topic, rqos, &listener, dds::core::status::StatusMask::data_available());
    // dds::sub::LoanedSamples<msgs::LowCmd> samples;

    while (running)
    {

        // /* Try taking samples from the reader. */
        // samples = reader.take();

        // /* Are samples read? */
        // if (samples.length() > 0) {
        //     /* Use an iterator to run over the set of samples. */
        //     dds::sub::LoanedSamples<msgs::LowCmd>::const_iterator sample_iter;
        //     for (sample_iter = samples.begin();
        //             sample_iter < samples.end();
        //             ++sample_iter) {
        //         /* Get the message and sample information. */
        //         const msgs::LowCmd& msg = sample_iter->data();
        //         const dds::sub::SampleInfo& info = sample_iter->info();
        //         std::cout << "go2py_callback got a sample..." << std::endl;
        //     }
        // }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "go2py_callback stopped." << std::endl;
}

void Bridge::InitLowCmd()
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

void Bridge::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void Bridge::LowCmdWrite()
{
    low_cmd.motor_cmd()[2].q() = 0;
    low_cmd.motor_cmd()[2].dq() = 0;
    low_cmd.motor_cmd()[2].kp() = 0;
    low_cmd.motor_cmd()[2].kd() = 0;
    low_cmd.motor_cmd()[2].tau() = 0;
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    lowcmd_publisher->Write(low_cmd);
}

class Go2PyListener: public dds::sub::NoOpDataReaderListener<msgs::LowCmd>
{
  public:
    using callback_func = std::function<bool(dds::sub::DataReader<msgs::LowCmd>&,
                                             dds::pub::DataWriter<msgs::LowState>&)>;
    Go2PyListener() = delete;
    Go2PyListener( const callback_func &f):
        dds::sub::DataReaderListener<msgs::LowCmd>(), _f(f) { ; }

    void on_data_available(dds::sub::DataReader<msgs::LowCmd>& rd,
                           dds::pub::DataWriter<msgs::LowState>& rw) {
        (void)_f(rd, rw);
    }
  private:
    callback_func _f; // Private member variable to store the callback function
};

static bool data_available(dds::sub::DataReader<msgs::LowCmd>& rd,
                           dds::pub::DataWriter<msgs::LowState>& rw)
{
    return true;
}
int main(int argc, const char** argv)
{
    dds::domain::DomainParticipant participant(domain::default_id());

    dds::topic::qos::TopicQos tqos;
    tqos << dds::core::policy::Reliability::Reliable(dds::core::Duration::from_secs(10));
    dds::topic::Topic<msgs::LowCmd> cmd_topic(participant, "go2py/lowcmd", tqos);
    dds::topic::Topic<msgs::LowState> state_topic(participant, "go2py/lowstate", tqos);

    dds::pub::qos::PublisherQos pqos;
    pqos << dds::core::policy::Partition("pong");
    dds::pub::Publisher publisher(participant, pqos);

    dds::sub::qos::SubscriberQos sqos;
    sqos << dds::core::policy::Partition("ping");
    dds::sub::Subscriber subscriber(participant, sqos);

    dds::pub::qos::DataWriterQos wqos;
    wqos << dds::core::policy::WriterDataLifecycle::ManuallyDisposeUnregisteredInstances();
    dds::pub::DataWriter<msgs::LowState> writer(publisher, state_topic, wqos);

    Go2PyListener listener(&data_available);

    dds::sub::DataReader<msgs::LowCmd>
      reader(
        subscriber,
        cmd_topic,
        dds::sub::qos::DataReaderQos(),
        &listener,
        dds::core::status::StatusMask::data_available());

    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    Bridge bridge;
    bridge.Init();
    while (1)
    {
        sleep(10);
    }

    return 0;
}
