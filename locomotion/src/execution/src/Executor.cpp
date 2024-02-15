#include "Executor.hpp"
#include <chrono>
#include <thread>

Executor::Executor() : m_name("svan") {
    InitClass();
}
Executor::Executor(const std::string& name) : m_name(name) {
    InitClass();
}

Executor::~Executor() {
    if (m_comm_thread.joinable()) {
        m_comm_thread.join();
    }
}

void Executor::setUpdateRate(const float& rate) {
    m_rate = rate;
    m_dt = 1./m_rate;
}

void Executor::InitClass() {
    std::string urdf_filepath = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/" + m_name + "_description/urdf/" + m_name + "_description.urdf";
    m_robot = Quadruped(urdf_filepath);
    std::cout << m_robot.getURDFFilepath() << "\n";
    m_robot.setStanceHeight(0.34);
    m_robot.showParams();
    m_rate = 1000;
    m_dt = 1./m_rate;

    m_plant_data_ptr = std::make_shared<QuadROSComm>(m_name);
    // m_plant_data->setAccessMode(DATA_ACCESS_MODE::EXECUTOR_TO_PLANT);
    m_plant_data_ptr->setSensorDataPtr(&m_sensor_data);
    m_plant_data_ptr->setCommandDataPtr(&m_cmd_data);

    m_estimator.SetRobot(m_robot);
    m_estimator.setSensorDataPtr(&m_sensor_data);
    m_estimator.setEstimationDataPtr(&m_est_data);
    m_estimator.setPlannerDataPtr(&m_planner_data);

    m_controller.SetRobot(m_robot);
    // m_controller.startFromSleep(false);
    m_controller.setEstimationDataPtr(&m_est_data);
    m_controller.setPlannerDataPtr(&m_planner_data);
    m_controller.setJointCommandDataPtr(&m_cmd_data);

    m_planner.SetRobot(m_robot);
    m_planner.setPlannerDataPtr(&m_planner_data);
    m_planner.setEstimationDataPtr(&m_est_data);

    Gait trot(Gait::TROT);
    // trot.SetParams(0.5, 0, 0.09, Eigen::Vector4d(0.3, 0.3, 0.3, 0.3), Eigen::Vector4d(0.75, 0.5, 0, 0.25));
    trot.SetParams(0.5, 0, 0.09, Eigen::Vector4d(0.5, 0.5, 0.5, 0.5), Eigen::Vector4d(0.5, 0, 0, 0.5));
    // trot.ShowParams();

    Gait stance(Gait::STANCE);

    Gait walk(Gait::STANCE);
    walk.SetParams(5.0, 0, 0.09, Eigen::Vector4d(0.9, 0.9, 0.9, 0.9), Eigen::Vector4d(0.75, 0.25, 0.5, 0.0));

    m_planner.SetGait(trot);
    m_planner.SetDesiredVelocity(0.5, 0.0, 0.0);

    m_planner_data.x = m_robot.getStanceStates();
}

void Executor::step() {
    Timer timer("Executor Step");
    // m_plant_data.getSensorData(m_sensor_data);
    m_plant_data_ptr->getSensorData(m_sensor_data);
    {
        Timer timer1("Planner step");
        m_planner.Step();
    }
    std::cout << "Planner states: " << m_planner_data.x.transpose() << "\n";
    // std::cout << "Planner vel: " << m_planner_data.xd.transpose() << "\n";
    std::cout << "tau_est: " << m_sensor_data.tau.transpose() << "\n";
    {
        Timer time1("Estimation step");
        m_estimator.Step();
    }
    // std::cout << "contact state: " << m_est_data.cs.transpose() << "\n";
    {
        Timer timer1("Controller step");
        m_controller.Step();
    }
    // std::cout << "quat: " << m_sensor_data.quat.transpose() << "\n";
    // std::cout << "Cmd js: " << m_cmd_data.q.transpose() << "\n";
    // m_plant_data.writeCommandData(m_cmd_data);
    m_plant_data_ptr->writeCommandData(m_cmd_data);

    // Sleep for specified time to maintain update rate
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

void Executor::run() {
    delay_ms = int(m_dt * double(1e3));

    m_comm_thread = std::thread(&QuadROSComm::Run, m_plant_data_ptr);
    m_comm_thread.detach();

    while (true) {
        step();
    }
}