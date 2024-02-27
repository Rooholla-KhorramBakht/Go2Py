#include "QuadROSComm.hpp"

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    std::string robot_name("go2");

    QuadrupedCommandData cmd_data;
    QuadrupedSensorData sensor_data;
    auto comm_obj = std::make_shared<QuadROSComm>(robot_name);
    comm_obj->setSensorDataPtr(&sensor_data);
    comm_obj->setCommandDataPtr(&cmd_data);
    // QuadROSComm comm_obj(robot_name);

    comm_obj->Run();

    return 0;
}