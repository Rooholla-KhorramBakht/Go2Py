#include "Executor.hpp"
#include <csignal>

Executor* quad_exec_ptr;

// Signal handler to gracefully handle Ctrl+C
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    // quad_exec_ptr->~Executor();
    // Add cleanup or exit logic as needed

    // if (quad_exec_ptr) {
    // }

    delete quad_exec_ptr;

    exit(signum);
}

int main(int argc, char** argv) {

#if defined(USE_ROS_COMM)
    std::cout << "Using ROS...\n";
    rclcpp::init(argc, argv);
#elif defined(USE_DDS_COMM)
    std::cout << "Using DDS...\n";
#else
    std::cout << "Using SHM...\n";
#endif

    if (argc != 2) {
        std::cerr << "Usage: ./quad_sim [robot_name]\n";
        return 1;
    }

    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    std::string m_name = std::string(argv[1]);
    std::string robot_name = m_name;
    // quad_exec_ptr = std::make_shared<Executor>(robot_name);
    quad_exec_ptr = new Executor(robot_name);

    quad_exec_ptr->setUpdateRate(1000);
    quad_exec_ptr->run();

    return 0;

#if defined(USE_ROS_COMM)
    rclcpp::shutdown();
#endif
}