#pragma once

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// Libraries for sleep
#include <chrono>
#include <thread>

#include <sys/ipc.h>
#include <sys/shm.h>
#include "memory_types.hpp"

class MJSim {
public:
    MJSim();
    ~MJSim();

    virtual void CustomController(const mjModel* model, mjData* data);

    void Run();
private:
    void init_sim();

    // keyboard callback
    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

    // mouse button callback
    static void mouse_button(GLFWwindow* window, int button, int act, int mods);

    // mouse move callback
    static void mouse_move(GLFWwindow* window, double xpos, double ypos);

    // scroll callback
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);

    void UpdateSensorData();

    std::string m_name;

    // MuJoCo data structures
    mjModel* m = NULL;                  // MuJoCo model
    mjData* d = NULL;                   // MuJoCo data
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context
    GLFWwindow* window;

    // mouse interaction
    bool button_left = false;
    bool button_middle = false;
    bool button_right =  false;
    double lastx = 0;
    double lasty = 0;

    // holders of one step history of time and position to calculate dertivatives
    mjtNum position_history = 0;
    mjtNum previous_time = 0;

    // controller related variables
    float_t ctrl_update_freq = 100;
    mjtNum last_update = 0.0;
    mjtNum ctrl;

    // Variables for SHM read (Control commands: joint position, velocity, torque)
    key_t read_key;
    QuadrupedCommandData joint_command_data;
    // Variables for SHM write (Sensor data: joint position, velocity, torque, imu data and probably base pose and contact states if required)
    key_t write_key;
    QuadrupedSensorData sensor_data;
};