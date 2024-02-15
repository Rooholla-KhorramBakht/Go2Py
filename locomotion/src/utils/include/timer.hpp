#ifndef __TIMER_HH__
#define __TIMER_HH__

#include <chrono>
#include <iostream>

class Timer {
    public:
        Timer() : Timer("") {
        }

        Timer(const char* name) : m_Name(name), m_Stopped(false) {
            m_StartTimePoint = std::chrono::high_resolution_clock::now();
        }

        ~Timer() {
            if (!m_Stopped) {
                Stop(); 
            }
        }
    private:
        const char* m_Name;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_StartTimePoint;
        bool m_Stopped;

        void Stop() {
            auto endTimePont = std::chrono::high_resolution_clock::now();
            auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimePoint).time_since_epoch().count();
            auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimePont).time_since_epoch().count();
            auto duration = end - start;

            double ms = duration * 0.001;
            std::cout << m_Name << ": " << duration << "us (" << ms << "ms)\n";

            m_Stopped = true;
        }
};

#endif