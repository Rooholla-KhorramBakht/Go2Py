#include <iostream>
#include "go2py/LowCmd.hpp"
#include <thread>
#include "utils/dds_subscriber.hpp"
#include "utils/dds_publisher.hpp"


// User-defined callback function
void userDefinedCallback(const msgs::LowCmd& msg) {
    // Do something with the received message
    std::cout << "User callback executed with message: " << std::endl;
}

int main() {
    // Create a subscriber with a topic name and a user callback
    DDSSubscriber<msgs::LowCmd> subscriber("go2py/lowcmd", userDefinedCallback);
    
    // Create a publisher for the LowCmd message type on the topic "LowCmdTopic"
    DDSPublisher<msgs::LowCmd> publisher("LowCmdTopic");

    // Create a LowCmd message
    msgs::LowCmd message;

    // Initialize the message data
    for(int i = 0; i < 12; i++) {
        message.q()[i] = 0.0f;
        message.dq()[i] = 0.0f;
        message.tau_ff()[i] = 0.0f;
        message.kp()[i] = 1.0f;
        message.kv()[i] = 0.5f;
    }
    message.e_stop() = 0;
    
    while(1)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        publisher.publish(message);
    }
    return 0;
}
