#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// Libraries for sleep
#include <chrono>
#include <thread>

#include "memory_types.hpp"
#ifdef USE_ROS_COMM
#include "QuadROSComm.hpp"
#else
#include "SHM.hpp"
#endif

#define COMPILE_WITH_VISUALIZATION

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data

#ifdef COMPILE_WITH_VISUALIZATION
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
#endif

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;
mjtNum timezero = 0;

// controller related variables
float_t ctrl_update_freq = 500;
mjtNum last_update = 0.0;
mjtNum ctrl;

// // Variables for SHM read (Control commands: joint position, velocity, torque)
// key_t read_key;
// QuadrupedCommandData joint_command_data;
// // Variables for SHM write (Sensor data: joint position, velocity, torque, imu data and probably base pose and contact states if required)
// key_t write_key;
// QuadrupedSensorData sensor_data;

QuadrupedSensorData sensor_data;
QuadrupedCommandData joint_command_data;
QuadrupedMeasurementData measurement_data;
// SHM comm_data(DATA_ACCESS_MODE::PLANT_TO_EXECUTOR);
#ifdef USE_ROS_COMM
std::shared_ptr<QuadROSComm> comm_data_ptr;
#else
std::shared_ptr<SHM> comm_data_ptr;
#endif


#ifdef COMPILE_WITH_VISUALIZATION
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}
#endif

void UpdateSensorData();
void CustomController(const mjModel*, mjData*);

void init_model(std::string model_file) {
    // load and compile model
    char error[1000] = "Could not load binary model";
    if (std::strlen(model_file.c_str())>4 && !std::strcmp(model_file.c_str() + std::strlen(model_file.c_str())-4, ".mjb")) {
        m = mj_loadModel(model_file.c_str(), 0);
    } else {
        m = mj_loadXML(model_file.c_str(), 0, error, 1000);
    }
    if (!m) {
        mju_error("Load model error: %s", error);
    }
    // make data
    d = mj_makeData(m);
}

#ifdef COMPILE_WITH_VISUALIZATION
void init_visualizer() {
    // init GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
}
#endif

void run_simulation(float sim_time = 1, bool show_visualization=false, float fps=60.) {
#ifndef COMPILE_WITH_VISUALIZATION
    if (show_visualization) {
        std::cerr << "Exec compiled with Visuzlization disabled. Re-compile with COMPILE_WITH_VISUALIZATION flag enabled to start visualization!\n";
    }
#endif
#ifdef COMPILE_WITH_VISUALIZATION
    if (show_visualization) {
        init_visualizer();
        // use the first while condition if you want to simulate for a period.
        while( !glfwWindowShouldClose(window) && d->time-timezero < sim_time) {
        // while( !glfwWindowShouldClose(window)) {
            // advance interactive simulation for 1/60 sec
            //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
            //  this loop will finish on time for the next frame to be rendered at 60 fps.
            //  Otherwise add a cpu timer and exit this loop when it is time to render.
#endif
            mjtNum simstart = d->time;
#ifdef COMPILE_WITH_VISUALIZATION
            while( d->time - simstart < 1.0/fps ) {
#else
            while (d->time - simstart < sim_time) {
#endif
                UpdateSensorData();
                mj_step(m, d);
            }
#ifdef COMPILE_WITH_VISUALIZATION
            // 15 ms is a little smaller than 60 Hz.
            int delay_ms = int(double(1e3)/fps);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            // get framebuffer viewport
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

            // update scene and render
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);

            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(window);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();

        }
    } else {
        mjtNum simstart = d->time;
        float dt = 0;
        if (fps == 0) {
            dt = sim_time;
        } else {
            dt = 1./fps;
        }
        while (d->time - simstart < dt) {
            UpdateSensorData();
            mj_step(m, d);
        }
    }
#endif
}

void shutdown_simulation() {
    // free visualization storage
#ifdef COMPILE_WITH_VISUALIZATION
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
#endif

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);

#ifdef COMPILE_WITH_VISUALIZATION
    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
#endif
}