#include "Executor.hpp"
#include <chrono>
#include <thread>
#include <filesystem>

#if defined(USE_ROS_COMM)
#include "QuadROSComm.hpp"
#elif defined(USE_DDS_COMM)
#include "QuadDDSComm.hpp"
#else
#include "SHM.hpp"
#endif

Executor::Executor() : m_name("svan") {
    initClass();
}
Executor::Executor(const std::string& name) : m_name(name) {
    initClass();
}

Executor::~Executor() {
    m_planner.setStance();
    
    float shutdown_timer = 0;

    float shutdown_start = t_curr;
    bool shutdown_complete = false;
    
    while (shutdown_timer < 5) {
        t_last = t_curr;
        t_curr = updateTimer();
        m_dt = t_curr - t_last;

        m_planner.stanceToSleep();

        step();

        // Sleep for specified time to maintain update rate

        float t_end = updateTimer();
        int delay_time = delay_ms - (t_end - t_curr) * 1e3;
        if (delay_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        }
        shutdown_timer = updateTimer() - shutdown_start;
    }
}

void Executor::setUpdateRate(const float& rate) {
    m_rate = rate;
    m_dt = 1./m_rate;
    // std::cout << "Updated loop rate to: " << m_rate << "Hz\n";
    // InitClass();
}

void Executor::initClass() {
    std::string rel_model_path = "src/robots/" + m_name + "_description/urdf/" + m_name + "_description.urdf";
    std::filesystem::path filepath = std::filesystem::current_path().parent_path().parent_path() / rel_model_path;
    std::string urdf_filepath = filepath.string();
    m_robot = Quadruped(urdf_filepath);
    std::cout << m_robot.getURDFFilepath() << "\n";
    m_robot.setStanceHeight(0.34);
    m_robot.showParams();
    m_rate = 1000;
    m_dt = 1./m_rate;

#if defined(USE_ROS_COMM)
    m_plant_data_ptr = std::make_shared<QuadROSComm>(m_name, DATA_ACCESS_MODE::EXECUTOR);
    m_plant_data_ptr->setUpdateRate(1000);
    // m_plant_data_ptr->initClass();
    m_plant_data_ptr->setSensorDataPtr(&m_sensor_data);
    m_plant_data_ptr->setCommandDataPtr(&m_cmd_data);
    m_plant_data_ptr->setEstimationDataPtr(&m_est_data);
#elif defined(USE_DDS_COMM)
    m_plant_data_ptr = std::make_shared<QuadDDSComm>(m_name, DATA_ACCESS_MODE::EXECUTOR);
    m_plant_data_ptr->setUpdateRate(1000);
    // m_plant_data_ptr->initClass();
    m_plant_data_ptr->setSensorDataPtr(&m_sensor_data);
    m_plant_data_ptr->setCommandDataPtr(&m_cmd_data);
    m_plant_data_ptr->setEstimationDataPtr(&m_est_data);
#else
    m_plant_data_ptr = std::make_shared<SHM>(m_name, DATA_ACCESS_MODE::EXECUTOR);
#endif
    
    m_plant_data_ptr->setMeasurementDataPtr(&m_measurement_data);
    m_plant_data_ptr->setJoystickDataPtr(&m_joystick_data);

    m_estimator.SetRobot(m_robot);
    m_estimator.setLoopRate(m_rate);
    m_estimator.setSensorDataPtr(&m_sensor_data);
    m_estimator.setEstimationDataPtr(&m_est_data);
    m_estimator.setPlannerDataPtr(&m_planner_data);
    m_estimator.setMeasurementDataPtr(&m_measurement_data);

    m_controller.SetRobot(m_robot);
    m_controller.startFromSleep(true);
    m_controller.setEstimationDataPtr(&m_est_data);
    m_controller.setPlannerDataPtr(&m_planner_data);
    m_controller.setJointCommandDataPtr(&m_cmd_data);

    m_planner.setRobot(m_robot);
    m_planner.startFromSleep(true);
    m_planner.setPlannerDataPtr(&m_planner_data);
    m_planner.setEstimationDataPtr(&m_est_data);

    trot = Gait(Gait::TROT);
    // trot.SetParams(0.5, 0, 0.09, Eigen::Vector4d(0.3, 0.3, 0.3, 0.3), Eigen::Vector4d(0.75, 0.5, 0, 0.25));
    trot.SetParams(0.5, 0, 0.09, Eigen::Vector4d(0.5, 0.5, 0.5, 0.5), Eigen::Vector4d(0.5, 0, 0, 0.5));
    // trot.SetParams(0.5, 0, 0.09, Eigen::Vector4d(0.6, 0.6, 0.6, 0.6), Eigen::Vector4d(0.5, 0, 0, 0.5));
    // trot.ShowParams();

    stance = Gait(Gait::STANCE);

    Gait walk(Gait::TROT);
    walk.SetParams(5.0, 0, 0.09, Eigen::Vector4d(0.9, 0.9, 0.9, 0.9), Eigen::Vector4d(0.75, 0.25, 0.5, 0.0));
    // set velocity to ~0.05 to work with gait period of 5

    m_planner.reset();
    m_planner.setGait(stance);
    m_planner.setDesiredVelocity(0.0, 0.0, 0.0);

    // m_planner_data.x = m_robot.getStanceStates();
    m_planner_data.x = m_robot.getSleepStates();

    m_startTimePoint = std::chrono::high_resolution_clock::now();
    m_lastTimePoint = std::chrono::high_resolution_clock::now();
    m_currentTimePoint = std::chrono::high_resolution_clock::now();
}

double Executor::updateTimer() {
    auto currTimePont = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTimePoint).time_since_epoch().count();
    auto curr = std::chrono::time_point_cast<std::chrono::microseconds>(currTimePont).time_since_epoch().count();
    auto duration = curr - start;

    return duration * double(1e-6);
}

void Executor::step() {
    // Timer timer("Executor Step");
    {
        // Timer timer1("Reading sensor data");
        m_plant_data_ptr->getSensorData(m_sensor_data);
    }
    {
        // Timer timer1("Planner step");
        m_planner.step(m_dt, t_curr);
    }
    {
        // Timer time1("Estimation step");
        m_estimator.Step(m_dt);
    }
    {
        // Timer timer1("Controller step");
        m_controller.Step();
    }
    {
        // Timer timer1("Write command data");
        m_plant_data_ptr->writeCommandData(m_cmd_data);
    }
}

void Executor::run() {
    delay_ms = int(m_dt * double(1e3));
#if defined(USE_ROS_COMM)
    m_plant_data_ptr->start_thread();
#elif defined(USE_DDS_COMM)
    m_plant_data_ptr->start_thread();
#endif

    while (true) {
        t_last = t_curr;
        t_curr = updateTimer();
        m_dt = t_curr - t_last;

        // if (t_curr < 5) {
        //     m_planner.setGait(trot);
        //     m_planner.setDesiredVelocity(0.5, 0.0, 0.0);
        //     // m_planner.setTargetBasePosition(vec3(5, 0, 0), 0, vec3(1.0, 1.0, 0.0));
        // } else if (t_curr < 10) {
        //     m_planner.setDesiredVelocity(0.0, 0.5, 0.0);
        // } else if (t_curr < 15) {
        //     m_planner.setDesiredVelocity(-0.5, 0.0, 0.0);
        // } else 
        // if (t_curr < 50) {
        
        // if (t_curr < 15) {
        if(t_curr < 5) {  
        // if(!m_planner.setTargetBasePosition(vec3(0, 0, 0.34), 0, vec3(0, 0, 0.2))) {
            // std::cout << "Reached\n";
            m_planner.sleepToStance();
        } else if (t_curr < 8) {
            // m_planner.setGait(trot);
            m_planner.setGait(trot);
            m_planner.setDesiredVelocity(0.2, 0, 0.0);
            // m_planner.setTargetBasePosition(vec3(5.0, 0, 0.34), 0.0);
            // std::cout << "Reaching\n";
        } else if (t_curr < 9) {
            m_planner.setGait(stance);
        } else {
            break;
        }
        // }

        // if(m_joystick_data.mode == 0) {
        //     m_planner.sleepToStance();
        // } else if (m_joystick_data.mode == 1) {
        //     m_planner.stanceToSleep();
        // } else if (m_joystick_data.mode == 2) {
        //     if (m_joystick_data.gait == 0) {
        //         m_planner.setGait(stance);
        //     } else if (m_joystick_data.gait == 1) {
        //         m_planner.setGait(trot);
        //         float vx = m_joystick_data.vel_x;
        //         float vy = m_joystick_data.vel_y;
        //         float vyaw = m_joystick_data.vel_yaw;
        //         m_planner.setDesiredVelocity(vx, vy, vyaw);
        //     }
        // }
            // m_planner.setDesiredVelocity(0.5, 0.0, 0.4);
            // m_planner.setTargetBasePosition(vec3(-5, 0, 0), 0.5, vec3(1.0, 1.0, 0.0));
        // } else if (t_curr < 20) {
        //     m_planner.setDesiredVelocity(-0.5, 0, 0);
            // m_planner.setTargetBasePosition(vec3(0, 5, 0), 0, vec3(0.2, 0.2, 0.0));
        // } else if (t_curr < 25) {
        //     m_planner.setDesiredVelocity(0, -0.2, 0);
        //     // m_planner.setTargetBasePosition(vec3(0, 0, 0), 0, vec3(0.2, 0.2, 0.0));
        // } else {
        //     m_planner.setGait(stance);
        //     m_planner.setDesiredVelocity(0, 0, 0);
        // }

        step();

        // Sleep for specified time to maintain update rate
        // std::cout << "time: " << t_curr << "\n";
        // std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        float t_end = updateTimer();
        int delay_time = delay_ms - (t_end - t_curr) * 1e3;
        if (delay_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        }
    }
}