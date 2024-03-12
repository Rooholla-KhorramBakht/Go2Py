#include "QuadDDSComm.hpp"

QuadDDSComm::QuadDDSComm(const DATA_ACCESS_MODE& mode) : m_name("m2"), CommunicationManager("m2", mode) {
    initClass();
}
QuadDDSComm::QuadDDSComm(const std::string& name, const DATA_ACCESS_MODE& mode) : m_name(name), CommunicationManager(name, mode) {
    initClass();
}
QuadDDSComm::~QuadDDSComm() {
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void QuadDDSComm::initClass() {
    m_participant_ptr = std::make_shared<dds::domain::DomainParticipant>(domain::default_id());
    m_g2p_low_cmd_topic_ptr = std::make_shared<dds::topic::Topic<Go2pyLowCmd_>>(*m_participant_ptr, "rt/" + m_name + "/lowcmd");

    // ACCESS TYPE EXECUTOR
    m_g2p_low_cmd_pub_ptr = std::make_shared<dds::pub::Publisher>(*m_participant_ptr);
    m_g2p_low_cmd_writer_ptr = std::make_shared<dds::pub::DataWriter<Go2pyLowCmd_>>(*m_g2p_low_cmd_pub_ptr, *m_g2p_low_cmd_topic_ptr);

    // m_low_state_sub_ptr = std::make_shared<dds::sub::Subscriber>(*m_participant_ptr);
    // m_low_state_reader_ptr = std::make_shared<dds::sub::DataReader<LowState_>>(*m_low_state_sub_ptr, *m_low_state_topic_ptr);

    if (m_mode == DATA_ACCESS_MODE::PLANT) {
        m_low_state_pub_ptr.reset(new DDSPublisher<LowState_>("rt/lowstate"));

        m_low_cmd_sub_ptr.reset(new DDSSubscriber<LowCmd_>(
                "rt/lowcmd",
                std::bind(&QuadDDSComm::get_low_cmd_data, this, std::placeholders::_1)
            )
        );

        m_g2p_low_state_sub_ptr = NULL;

    } else if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        m_low_state_pub_ptr = NULL;
        m_low_cmd_sub_ptr = NULL;

        m_g2p_low_state_sub_ptr.reset(new DDSSubscriber<Go2pyLowState_>(
                "rt/" + m_name + "/lowstate", 
                std::bind(&QuadDDSComm::get_g2p_low_state_cb, this, std::placeholders::_1)
            )
        );
    }
    
}

void QuadDDSComm::get_low_cmd_data(const LowCmd_& low_cmd_msg) {
    for (int i = 0; i < 12; ++i) {
        m_joint_command_data_ptr->q(i) = low_cmd_msg.motor_cmd()[i].q();
        m_joint_command_data_ptr->qd(i) = low_cmd_msg.motor_cmd()[i].dq();
        m_joint_command_data_ptr->tau(i) = low_cmd_msg.motor_cmd()[i].tau();
        m_joint_command_data_ptr->kp(i) = low_cmd_msg.motor_cmd()[i].kp();
        m_joint_command_data_ptr->kd(i) = low_cmd_msg.motor_cmd()[i].kd();
    }
}

void QuadDDSComm::get_g2p_low_state_cb(const Go2pyLowState_& msg) {
    for (int i = 0; i < 12; ++i) {
        m_sensor_data_ptr->q(i) = msg.q()[i];
        m_sensor_data_ptr->qd(i) = msg.dq()[i];
        m_sensor_data_ptr->tau(i) = msg.tau_est()[i];
    }
    for (int i = 0; i < 4; ++i) {
        m_sensor_data_ptr->quat(i) = msg.quat()[i];
    }
    for (int i = 0; i < 3; ++i) {
        m_sensor_data_ptr->a_B(i) = msg.accel()[i];
        m_sensor_data_ptr->w_B(i) = msg.gyro()[i];
    }
}

void QuadDDSComm::write_low_state_data() {
    // const auto& low_state_msg = LowState_();
    for (int i = 0; i < 12; ++i) {
        low_state_msg.motor_state()[i].q() = m_sensor_data_ptr->q(i);
        low_state_msg.motor_state()[i].dq() = m_sensor_data_ptr->qd(i);
        low_state_msg.motor_state()[i].tau_est() = m_sensor_data_ptr->tau(i);
    }
    for (int i = 0; i < 4; ++i) {
        low_state_msg.imu_state().quaternion()[i] = m_sensor_data_ptr->quat(i);
    }
    for (int i = 0; i < 3; ++i) {
        low_state_msg.imu_state().accelerometer()[i] = m_sensor_data_ptr->a_B(i);
        low_state_msg.imu_state().gyroscope()[i] = m_sensor_data_ptr->w_B(i);
    }

    // m_low_state_writer_ptr->write(low_state_msg);
    m_low_state_pub_ptr -> publish(low_state_msg);
}

void QuadDDSComm::write_low_cmd_data() {
    // const auto& go2py_low_cmd_msg = Go2pyLowCmd_();
    // if (m_joint_command_data_ptr != NULL) {
    //     std::cout << "Joint command pointer not set. Cannot write...\n";
    // }

    for (int i = 0; i < 12; ++i) {
        go2py_low_cmd_msg.q()[i] = m_joint_command_data_ptr->q(i);
        go2py_low_cmd_msg.dq()[i] = m_joint_command_data_ptr->qd(i);
        go2py_low_cmd_msg.tau()[i] = m_joint_command_data_ptr->tau(i);
        go2py_low_cmd_msg.kp()[i] = m_joint_command_data_ptr->kp(i);
        go2py_low_cmd_msg.kd()[i] = m_joint_command_data_ptr->kd(i);
    }

    m_g2p_low_cmd_writer_ptr->write(go2py_low_cmd_msg);
}

void QuadDDSComm::step() {
    if (m_mode == DATA_ACCESS_MODE::PLANT) {
        // get_low_cmd_data();
        write_low_state_data();
    } else if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        // get_low_state_data();
        write_low_cmd_data();
    }
}

void QuadDDSComm::run() {
    int delay_ms = int(m_dt * 1e3);
    // Make the termination more sound since it will be running on a different thread. Break the loop automatically when communication "ends".
    while (true) {
        step();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

void QuadDDSComm::start_thread() {
    m_thread = std::thread(&QuadDDSComm::run, this);
    m_thread.detach();
}