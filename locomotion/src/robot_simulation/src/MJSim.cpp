#include "MJSim.hpp"

MJSim::MJSim() : m_name("go2") {
    // Load and compile model
    init_sim();
}

MJSim::~MJSim() {
    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}

void MJSim::CustomController(const mjModel* model, mjData* data) {
    // controller with sensor readings
    if (previous_time == 0)
    {
        previous_time = data->time;
        return;
    }
    if (data->time - last_update > 1.0/ctrl_update_freq)
    {
        // Get the velocity of joint directly from MuJoCo?
        mjtNum vel = (data->sensordata[0] - position_history)/(d->time-previous_time);

        for (int i = 0; i < 12; ++i) {
            float dtheta = joint_command_data.q(i) - sensor_data.qd(i);
            float dtheta_d = joint_command_data.qd(i) - sensor_data.qd(i);
            data->ctrl[i] = joint_command_data.kp(i) * dtheta * joint_command_data.kd(i) * dtheta_d;
        }
        last_update = data->time;
        position_history = data->sensordata[0];
        previous_time = data->time;
    }
}
void MJSim::Run() {
    // use the first while condition if you want to simulate for a period.
    // while( !glfwWindowShouldClose(window) and d->time-timezero < 1.5)
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 ) {
            UpdateSensorData();
            mj_step(m, d);
        }

        // 15 ms is a little smaller than 60 Hz.
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
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
}

void MJSim::init_sim() {
    std::string model_file = "../../robots/" + m_name + "_description/mujoco/" + m_name + ".xml";

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

    // install control callback
    mjcb_control = this->CustomController;

    // Initialize DOFs
    // d->qpos[0] = 1.57;

    // run main loop, target real-time simulation and 60 fps rendering
    mjtNum timezero = d->time;
    double_t update_rate = 0.01;

    // making sure the first time step updates the ctrl previous_time
    last_update = timezero-1.0/ctrl_update_freq;
}

// keyboard callback
void MJSim::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void MJSim::mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void MJSim::mouse_move(GLFWwindow* window, double xpos, double ypos) {
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
void MJSim::scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void MJSim::UpdateSensorData() {
    for (int i = 0; i < 12; ++i) {
        sensor_data.q(i) = d -> sensordata[i];
    }
}