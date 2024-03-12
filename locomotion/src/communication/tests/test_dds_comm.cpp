#include "QuadDDSComm.hpp"

int main(int argc, char** argv) {

    std::string robot_name("go2");

    QuadrupedCommandData cmd_data;
    QuadrupedSensorData sensor_data;
    auto comm_obj = std::make_shared<QuadDDSComm>(robot_name, DATA_ACCESS_MODE::PLANT);
    comm_obj->setSensorDataPtr(&sensor_data);
    comm_obj->setCommandDataPtr(&cmd_data);
    // QuadROSComm comm_obj(robot_name);

    comm_obj->start_thread();

    int a;
    std::cin >> a;

    return 0;
}