#include "SHM.hpp"

SHM::SHM() : m_name("svan"), CommunicationManager(DATA_ACCESS_MODE::PLANT_TO_EXECUTOR) {
    InitClass();
}

SHM::SHM(const DATA_ACCESS_MODE& mode) : m_name("svan"), CommunicationManager(mode) {
    InitClass();
}

SHM::SHM(const std::string& name, const DATA_ACCESS_MODE& mode) : m_name(name), CommunicationManager(name, mode) {
    InitClass();
}

SHM::~SHM() {
    shmdt(m_joint_command_data_ptr);
    shmdt(m_sensor_data_ptr);
}

void* SHM::getSHMPointer(const key_t& key, const size_t& totalSize) {
    // get the id of the shared memory block with the given key
    int shmid = shmget(key, totalSize, 0666|IPC_CREAT);

    // std::cout << "shmid: " << shmid << "\n";

    // Raise error if the shared memory could not be created/assigned
    if (shmid == -1) {
        perror("shmget");
        return (void*)(01);
    }
    // get pointer to the shared memory block
    void *sharedMemory = shmat(shmid, nullptr, 0);

    // Raise error if the returned pointer is not valid
    if (sharedMemory == reinterpret_cast<void*>(01)) {
        perror("shmat");
        return (void*)(01);
    }

    return sharedMemory;
}

void SHM::SetupMemory() {
    key_t key;
    size_t totalSize;
    void* sharedMemory;

    key = SHM_KEYS::COMMAND_DATA;
    totalSize = sizeof(QuadrupedCommandData);
    // std::cout << "Command data  size: " << totalSize << "\n";
    sharedMemory = getSHMPointer(key, totalSize);
    m_joint_command_data_ptr = static_cast<QuadrupedCommandData*>(sharedMemory);

    key = SHM_KEYS::SENSOR_DATA;
    totalSize = sizeof(QuadrupedSensorData);
    // std::cout << "Sensor data  size: " << totalSize << "\n";
    sharedMemory = getSHMPointer(key, totalSize);
    m_sensor_data_ptr = static_cast<QuadrupedSensorData*>(sharedMemory);
    // std::cout << "Memory setup completed...\n";
}

void SHM::InitClass() {
    SetupMemory();
}