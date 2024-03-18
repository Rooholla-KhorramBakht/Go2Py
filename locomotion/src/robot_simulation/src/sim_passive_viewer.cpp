// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim_passive_viewer.hpp"
#include "utils.hpp"
#include <filesystem>
#include <csignal>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

// Signal handler to gracefully handle Ctrl+C
void signalHandler(int signum) {
    // std::cout << "Interrupt signal (" << signum << ") received.\n";
    // Add cleanup or exit logic as needed
    exit(signum);
}

void UpdateSensorData(mjData* data) {
    if (data == NULL) {
        return;
    }
    for (int i = 0; i < 12; ++i) {
        sensor_data.q(i) = data -> sensordata[i];
        sensor_data.qd(i) = data -> sensordata[12 + i];
        sensor_data.tau(i) = data -> sensordata[24 + i];
    }
    for (int i = 0; i < 3; ++i) {
        sensor_data.a_B(i) = data -> sensordata[36 + i];
        sensor_data.w_B(i) = data -> sensordata[39 + i];
    }
    for (int i = 0; i < 4; ++i) {
        sensor_data.quat(i) = data -> sensordata[(42 + ((i + 1) % 4))];
    }

    mat3x3 Rot = pinocchio::quat2rot(sensor_data.quat);
    sensor_data.a_W = Rot * sensor_data.a_B;
    sensor_data.w_W = Rot * sensor_data.w_B;

    // std::cout << "Quat: " << sensor_data.quat.transpose() << "\n";

    comm_data_ptr -> writeSensorData(sensor_data);

    // Updating measurement data
    for (int i = 0; i < 3; ++i) {
        measurement_data.base_position(i) = data -> sensordata[46 + i];
        measurement_data.base_velocity.linear(i) = data -> sensordata[49 + i];
        measurement_data.base_velocity.angular(i) = data -> sensordata[52 + i];
        measurement_data.base_acceleration.linear(i) = data -> sensordata[55 + i];
        measurement_data.base_acceleration.angular(i) = data -> sensordata[58 + i];
    }
    for (int i = 0; i < 4; ++i) {
        measurement_data.contact_force(3 * i) = -data -> sensordata[61 + 3 * i];
        measurement_data.contact_force(3 * i + 1) = -data -> sensordata[61 + 3 * i + 1];
        measurement_data.contact_force(3 * i + 2) = -data -> sensordata[61 + 3 * i + 2];
    }
    comm_data_ptr -> writeMeasurementData(measurement_data);
        
}

void CustomController(const mjModel* model, mjData* data) {
    // t_curr = updateTimer();
    t_curr = data->time;
    comm_data_ptr -> writePlantTime(t_curr);

    UpdateSensorData(data);
    // get joint command
    comm_data_ptr->getCommandData(joint_command_data);
    // controller with sensor readings
    // if (previous_time == 0) {
    //     previous_time = data->time;
    //     return;
    // }
    // std::cout << "cmd js: " << joint_command_data.q.transpose() << "\n";
    // std::cout << "kp: " << joint_command_data.kp.transpose() << "\n";
    // float Kp = 20;
    // float Kd = 0.5;
    float Kp = 60;
    float Kd = 5;
    // float Kp = 0;
    // float Kd = 0;
    // if (data->time - last_update > 1.0/ctrl_update_freq) {
        for (int i = 0; i < 12; ++i) {
            // Kp = joint_command_data.kp(i);
            // Kd = joint_command_data.kd(i);
            float dtheta = joint_command_data.q(i) - sensor_data.q(i);
            float dtheta_d = joint_command_data.qd(i) - sensor_data.qd(i);
            // data->ctrl[i] = joint_command_data.tau(i) + joint_command_data.kp(i) * dtheta * joint_command_data.kd(i) * dtheta_d;
            data->ctrl[i] = joint_command_data.tau(i) + Kp * dtheta + Kd * dtheta_d;
        }
    //     last_update = data->time;
    //     position_history = data->sensordata[0];
    //     previous_time = data->time;
    // }
}

// run event loop
int main(int argc, char** argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

#ifdef USE_ROS_COMM
    rclcpp::init(argc, argv);
#endif

    if (argc != 2) {
        std::cerr << "Usage: ./simulate_pv [robot_name]\n";
        return 1;
    }

    std::string m_name = std::string(argv[1]);

    // std::string model_file = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/" + m_name + "_description/mujoco/" + m_name + ".xml";
    // std::string model_file = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/" + m_name + "_description/mujoco/scene.xml";
    std::string rel_model_path = "src/robots/" + m_name + "_description/mujoco/scene.xml";
    std::filesystem::path filepath = std::filesystem::current_path().parent_path().parent_path() / rel_model_path;
    std::string model_file = filepath.string();

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    auto sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false
    );

    const char* filename = model_file.c_str();
    // if (argc >  1) {
    //     filename = argv[1];
    // }
    // std::cout << "Communication started. Starting physics thread...\n";

    // install control callback
    mjcb_control = CustomController;

    joint_command_data.q << 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3;

    // timezero = d->time;
    // double_t update_rate = 0.001;

    // making sure the first time step updates the ctrl previous_time
    // last_update = timezero-1.0/ctrl_update_freq;

    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

#if defined(USE_ROS_COMM)
    std::cout << "Using ROS for communication...\n";
    comm_data_ptr = std::make_shared<QuadROSComm>(m_name, DATA_ACCESS_MODE::PLANT);
    comm_data_ptr->setCommandDataPtr(&joint_command_data);
    comm_data_ptr->setSensorDataPtr(&sensor_data);
    comm_data_ptr->setMeasurementDataPtr(&measurement_data);
    // comm_data_ptr->setPlantTimePtr(&t_curr);
#elif defined(USE_DDS_COMM)
    comm_data_ptr = std::make_shared<QuadDDSComm>(m_name, DATA_ACCESS_MODE::PLANT);
    comm_data_ptr->setCommandDataPtr(&joint_command_data);
    comm_data_ptr->setSensorDataPtr(&sensor_data);
    comm_data_ptr->setMeasurementDataPtr(&measurement_data);
#else
    comm_data_ptr = std::make_shared<SHM>(m_name, DATA_ACCESS_MODE::PLANT);
#endif

#if defined(USE_ROS_COMM)
    // Run ros on separate non-blocking thread
    comm_data_ptr->start_thread();
#elif defined(USE_DDS_COMM)
    comm_data_ptr->start_thread();
#endif

    m_startTimePoint = std::chrono::high_resolution_clock::now();
    // start physics thread
    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
    // physicsthreadhandle.detach();
    
    // start simulation UI loop (blocking call)
    sim->RenderLoop();
    physicsthreadhandle.join();

#ifdef USE_ROS_COMM
    rclcpp::shutdown();
#endif

    return 0;
}
