#ifndef A05_ULTRASONIC_H
#define A05_ULTRASONIC_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <mutex>

class A05Ultrasonic {
public:
    A05Ultrasonic(const std::string& port, int baudrate);
    ~A05Ultrasonic();

    bool connect();
    void disconnect();
    bool isConnected() const;
    
    bool readData(std::vector<float>& distances, std::vector<bool>& valid);
    
    void setSimulationMode(bool simulate);
    void setSimulatedDistances(const std::vector<float>& distances);

private:
    serial::Serial serial_;
    std::string port_;
    int baudrate_;
    bool connected_;
    bool simulation_mode_;
    std::vector<float> simulated_distances_;
    std::mutex mutex_;
    
    bool readUARTData(std::vector<float>& distances, std::vector<bool>& valid);
    bool readRS485Data(std::vector<float>& distances, std::vector<bool>& valid);
    
    bool parseUARTData(const std::vector<uint8_t>& data, std::vector<float>& distances, std::vector<bool>& valid);
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);
    
    bool sendRS485Command(uint8_t command);
    bool parseRS485Data(const std::vector<uint8_t>& data, std::vector<float>& distances, std::vector<bool>& valid);
};

#endif // A05_ULTRASONIC_H