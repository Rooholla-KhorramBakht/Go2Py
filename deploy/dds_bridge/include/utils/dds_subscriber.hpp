#ifndef __DDS_SUBSCRIBER_HPP__
#define __DDS_SUBSCRIBER_HPP__

#include <iostream>
#include <functional>
#include <dds/dds.hpp>
#include <thread>
using namespace dds::sub;
using namespace dds::domain;
using namespace dds::topic;

template<typename T>
class DDSSubscriber {
    public:
        // Define a callback type for user-provided callbacks
        
        using UserCallback = std::function<void(const T&)>;

    private:
        DomainParticipant participant;
        Topic<T> topic;
        Subscriber subscriber;
        DataReader<T> reader;
        T latestMessage;
        UserCallback userCallback;

        // Internal Listener class
        class CustomListener : public NoOpDataReaderListener<msgs::LowCmd> {
        private:
            DDSSubscriber& parent;

        public:
            CustomListener(DDSSubscriber& parent) : parent(parent) {}

            void on_data_available(DataReader<T>& reader) override {
                // Take all available data
                dds::sub::LoanedSamples<T> samples = reader.take();
                if (samples.length() >0){
                    for (auto sample_iter = samples.begin();
                            sample_iter < samples.end();
                            ++sample_iter){
                                    parent.latestMessage = sample_iter->data(); // Update the latest message
                                    // If a user callback is provided, execute it
                                    if (parent.userCallback) {
                                        parent.userCallback(parent.latestMessage);
                                    }
                                
                    }
                }
            }
        };

    public:
        // Constructor
        DDSSubscriber(const std::string& topicName, UserCallback callback = nullptr, const int id = 0)
            : participant(id),
            topic(participant, topicName),
            subscriber(participant),
            userCallback(callback),
            reader(subscriber, topic, dds::sub::qos::DataReaderQos(), new CustomListener(*this), dds::core::status::StatusMask::data_available()) {}

        // Method to get the latest message
        msgs::LowCmd getLatestMessage() const {
            return latestMessage;
        }
};
#endif