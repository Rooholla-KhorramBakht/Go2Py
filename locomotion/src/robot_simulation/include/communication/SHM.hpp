#pragma once

#include "CommunicationManager.hpp"
#include <sys/ipc.h>
#include <sys/shm.h>

// Randomy chosen key enums
enum SHM_KEYS {
    SENSOR_DATA = 123,
    MEASUREMENT_DATA = 496,
    COMMAND_DATA = 549
};

class SHM : public CommunicationManager {
public:
    SHM();
    SHM(const DATA_ACCESS_MODE& mode);
    SHM(const std::string& name, const DATA_ACCESS_MODE& mode);
    ~SHM();

    // void writeSensorData(const QuadrupedSensorData& sensor_data) override;
    // void writeCommandData(const QuadrupedCommandData& cmd_data) override;
private:
    std::string m_name;

    void InitClass();

    void SetupMemory();
    void* getSHMPointer(const key_t&, const size_t&);
};