#include "ExecutorSHM.hpp"
// #include <csignal>

// // Signal handler to gracefully handle Ctrl+C
// void signalHandler(int signum) {
//     std::cout << "Interrupt signal (" << signum << ") received.\n";
//     // Add cleanup or exit logic as needed
//     exit(signum);
// }

int main(int argc, char** argv) {

    if (argc != 2) {
        std::cerr << "Usage: ./quad_sim [robot_name]\n";
        return 1;
    }

    // Register signal handler for Ctrl+C
    // signal(SIGINT, signalHandler);

    std::string m_name = std::string(argv[1]);
    std::string robot_name = m_name;
    Executor quad_exec(robot_name);

    quad_exec.setUpdateRate(1000);
    quad_exec.run();

    return 0;
}