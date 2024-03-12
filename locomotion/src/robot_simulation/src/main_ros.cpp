#include "mjsim_helper_ros.hpp"
#include "utils.hpp"
#include <filesystem>

#include <csignal>

// Signal handler to gracefully handle Ctrl+C
void signalHandler(int signum) {
    // std::cout << "Interrupt signal (" << signum << ") received.\n";
    // Add cleanup or exit logic as needed
    exit(signum);
}

void UpdateSensorData() {
    for (int i = 0; i < 12; ++i) {
        sensor_data.q(i) = d -> sensordata[i];
        sensor_data.qd(i) = d -> sensordata[12 + i];
        sensor_data.tau(i) = d -> sensordata[24 + i];
    }
    for (int i = 0; i < 3; ++i) {
        sensor_data.a_B(i) = d -> sensordata[36 + i];
        sensor_data.w_B(i) = d -> sensordata[39 + i];
    }
    for (int i = 0; i < 4; ++i) {
        sensor_data.quat(i) = d -> sensordata[(42 + ((i + 1) % 4))];
    }

    mat3x3 Rot = pinocchio::quat2rot(sensor_data.quat);
    sensor_data.a_W = Rot * sensor_data.a_B;
    sensor_data.w_W = Rot * sensor_data.w_B;

    // std::cout << "Quat: " << sensor_data.quat.transpose() << "\n";

    comm_data_ptr -> writeSensorData(sensor_data);

    // Updating measurement data
    for (int i = 0; i < 3; ++i) {
        measurement_data.base_position(i) = d -> sensordata[46 + i];
        measurement_data.base_velocity.linear(i) = d -> sensordata[49 + i];
        measurement_data.base_velocity.angular(i) = d -> sensordata[52 + i];
        measurement_data.base_acceleration.linear(i) = d -> sensordata[55 + i];
        measurement_data.base_acceleration.angular(i) = d -> sensordata[58 + i];
    }
    for (int i = 0; i < 4; ++i) {
        measurement_data.contact_force(3 * i) = -d -> sensordata[61 + 3 * i];
        measurement_data.contact_force(3 * i + 1) = -d -> sensordata[61 + 3 * i + 1];
        measurement_data.contact_force(3 * i + 2) = -d -> sensordata[61 + 3 * i + 2];
    }
    comm_data_ptr -> writeMeasurementData(measurement_data);
        
}

void CustomController(const mjModel* model, mjData* data) {
    // get joint command
    comm_data_ptr->getCommandData(joint_command_data);
    // controller with sensor readings
    if (previous_time == 0) {
        previous_time = data->time;
        return;
    }
    // std::cout << "cmd js: " << joint_command_data.q.transpose() << "\n";
    // float Kp = 20;
    // float Kd = 0.5;
    float Kp = 60;
    float Kd = 5;
    // float Kp = 26.16;
    // float Kd = 13.13;
    if (data->time - last_update > 1.0/ctrl_update_freq) {
        // Get the velocity of joint directly from MuJoCo?
        // mjtNum vel = (data->sensordata[0] - position_history)/(d->time-previous_time);

        for (int i = 0; i < 12; ++i) {
            float dtheta = joint_command_data.q(i) - sensor_data.q(i);
            float dtheta_d = joint_command_data.qd(i) - sensor_data.qd(i);
            // data->ctrl[i] = joint_command_data.kp(i) * dtheta * joint_command_data.kd(i) * dtheta_d;
            data->ctrl[i] = joint_command_data.tau(i) + Kp * dtheta + Kd * dtheta_d;
        }
        last_update = data->time;
        position_history = data->sensordata[0];
        previous_time = data->time;
    }
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    if (argc != 2) {
        std::cerr << "Usage: ./quad_sim [robot_name]\n";
        return 1;
    }

    std::string m_name = std::string(argv[1]);

    // std::string model_file = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/" + m_name + "_description/mujoco/" + m_name + ".xml";
    // std::string model_file = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/" + m_name + "_description/mujoco/scene.xml";
    std::string rel_model_path = "src/robots/" + m_name + "_description/mujoco/scene.xml";
    std::filesystem::path filepath = std::filesystem::current_path().parent_path().parent_path() / rel_model_path;
    std::string model_file = filepath.string();

    init_model(model_file);

    joint_command_data.q << 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3;

    // install control callback
    mjcb_control = CustomController;

    timezero = d->time;
    double_t update_rate = 0.001;

    // making sure the first time step updates the ctrl previous_time
    last_update = timezero-1.0/ctrl_update_freq;

    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    comm_data_ptr = std::make_shared<QuadROSComm>(m_name, DATA_ACCESS_MODE::PLANT);
    // comm_data_ptr -> setAccessMode(DATA_ACCESS_MODE::PLANT);
    comm_data_ptr->setSensorDataPtr(&sensor_data);
    comm_data_ptr->setCommandDataPtr(&joint_command_data);
    comm_data_ptr->setMeasurementDataPtr(&measurement_data);

    std::thread comm_thread;
    comm_thread = std::thread(&QuadROSComm::Run, comm_data_ptr);
    comm_thread.detach();
    
    // Run the simulation for 10 seconds with visualization enabled with 60 fps
    run_simulation(10000, true, 60);

    shutdown_simulation();

    return 0;
}