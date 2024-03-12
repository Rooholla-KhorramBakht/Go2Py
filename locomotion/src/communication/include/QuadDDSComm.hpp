#pragma once

#include "CommunicationManager.hpp"

#include <thread>
#include <chrono>
#include <string>

#include <dds/dds.hpp>

#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"

#include "Go2pyLowCmd.hpp"
#include "Go2pyLowState.hpp"
#include "LowState.hpp"
#include "LowCmd.hpp"

using namespace org::eclipse::cyclonedds;
using namespace unitree_go::msg::dds_;

class QuadDDSComm : public CommunicationManager {
public:
    QuadDDSComm(const DATA_ACCESS_MODE&);
    QuadDDSComm(const std::string&, const DATA_ACCESS_MODE&);
    ~QuadDDSComm();

    void setUpdateRate(const float& rate) {
        m_loop_rate = rate;
        m_dt = 1./rate;
    }

    void step();
    void run();
    void start_thread();

    void initClass();
private:
    std::string m_name;
    float m_loop_rate = 1000;
    float m_dt = 0.001;

    std::shared_ptr<dds::domain::DomainParticipant> m_participant_ptr = NULL;

    std::shared_ptr<dds::topic::Topic<LowState_>> m_low_state_topic_ptr = NULL;
    std::shared_ptr<dds::topic::Topic<Go2pyLowCmd_>> m_g2p_low_cmd_topic_ptr = NULL;
    std::shared_ptr<dds::topic::Topic<LowCmd_>> m_low_cmd_topic_ptr = NULL;

    std::shared_ptr<DDSSubscriber<Go2pyLowState_>> m_g2p_low_state_sub_ptr;
    std::shared_ptr<DDSPublisher<LowState_>> m_low_state_pub_ptr;
    std::shared_ptr<DDSSubscriber<LowCmd_>> m_low_cmd_sub_ptr;

    // ACCESS TYPE PLANT
    
    std::shared_ptr<dds::sub::Subscriber> m_g2p_low_cmd_sub_ptr = NULL;
    std::shared_ptr<dds::sub::DataReader<Go2pyLowCmd_>> m_g2p_low_cmd_reader_ptr = NULL;
    
    // ACCESS TYPE EXECUTOR
    std::shared_ptr<dds::pub::Publisher> m_g2p_low_cmd_pub_ptr = NULL;
    std::shared_ptr<dds::pub::DataWriter<Go2pyLowCmd_>> m_g2p_low_cmd_writer_ptr = NULL;



    bool get_g2p_low_cmd_data();
    void get_low_cmd_data(const LowCmd_& low_cmd_msg);
    // bool get_low_state_data();
    void write_low_cmd_data();
    void write_low_state_data();

    void get_g2p_low_state_cb(const Go2pyLowState_& msg);

    std::thread m_thread;

    // Data holders
    Go2pyLowCmd_ go2py_low_cmd_msg;
    LowState_ low_state_msg;
    Go2pyLowState_ g2p_low_state_msg;
};