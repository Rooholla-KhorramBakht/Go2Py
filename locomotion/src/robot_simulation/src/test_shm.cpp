#include "Communication/SHM.hpp"

int main(int argc, char** argv) {
    SHM shm_data(DATA_ACCESS_MODE::EXECUTOR_TO_PLANT);
    QuadrupedSensorData sensor_data;

    int i = 0;
    while (i < 500000) {
        shm_data.getSensorData(sensor_data);
        std::cout << "Quat: " << sensor_data.quat.transpose() << "\n";
        i++;
    }

    return 0;
}