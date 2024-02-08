#ifndef __DDS_PUBLISHER_HPP__
#define __DDS_PUBLISHER_HPP__

#include <iostream>
#include <functional>
#include <dds/dds.hpp>
#include <thread>

using namespace dds::sub;
using namespace dds::domain;
using namespace dds::topic;

template<typename T>
class DDSPublisher {
    private:
        dds::domain::DomainParticipant participant;
        dds::topic::Topic<T> topic;
        dds::pub::Publisher publisher;
        dds::pub::DataWriter<T> writer;

    public:
        // Constructor
        DDSPublisher(const std::string& topicName, const int id = 0)
            : participant(dds::domain::DomainParticipant(id)),
            topic(participant, topicName),
            publisher(dds::pub::Publisher(participant)),
            writer(dds::pub::DataWriter<T>(publisher, topic)) {}

        // Method to publish a message
        void publish(const T& message) {
            writer.write(message);
        }
};
#endif