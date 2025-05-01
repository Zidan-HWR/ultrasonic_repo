#include "a05_ultrasonic/a05_ultrasonic.h"
#include <ros/console.h>
#include <algorithm>

A05Ultrasonic::A05Ultrasonic(const std::string& port, int baudrate) 
    : port_(port), baudrate_(baudrate), connected_(false), simulation_mode_(false) {
    simulated_distances_ = {1.0f, 1.5f, 2.0f, 2.5f}; // Default simulation values
}

A05Ultrasonic::~A05Ultrasonic() {
    disconnect();
}

bool A05Ultrasonic::connect() {
    try {
        serial_.setPort(port_);
        serial_.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);
        serial_.open();
        
        if(serial_.isOpen()) {
            connected_ = true;
            ROS_INFO_STREAM("Connected to A05 ultrasonic sensor on port " << port_);
            return true;
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port " << port_ << ": " << e.what());
    }
    
    connected_ = false;
    return false;
}

void A05Ultrasonic::disconnect() {
    if(serial_.isOpen()) {
        serial_.close();
    }
    connected_ = false;
}

bool A05Ultrasonic::isConnected() const {
    return connected_;
}

void A05Ultrasonic::setSimulationMode(bool simulate) {
    simulation_mode_ = simulate;
}

void A05Ultrasonic::setSimulatedDistances(const std::vector<float>& distances) {
    simulated_distances_ = distances;
}

bool A05Ultrasonic::readData(std::vector<float>& distances, std::vector<bool>& valid) {
    if (simulation_mode_) {
        distances = simulated_distances_;
        valid.assign(simulated_distances_.size(), true);
        ROS_DEBUG("Simulation mode active");
        return true;
    }
    
    if (!connected_) {
        ROS_ERROR("Not connected to sensor!");
        return false;
    }

    ROS_DEBUG("Attempting to read UART data");
    if (readUARTData(distances, valid)) {
        ROS_DEBUG("UART read successful");
        return true;
    }
    
    ROS_DEBUG("UART failed, trying RS485");
    if (readRS485Data(distances, valid)) {
        ROS_DEBUG("RS485 read successful");
        return true;
    }

    ROS_ERROR("Both UART and RS485 read attempts failed");
    return false;
}
bool A05Ultrasonic::readUARTData(std::vector<float>& distances, std::vector<bool>& valid) {
    if (!connected_) return false;
    
    try {
        std::vector<uint8_t> data;
        size_t bytes_read = serial_.read(data, 10);
        
        if (bytes_read == 10) {
            return parseUARTData(data, distances, valid);
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Error reading from serial port: " << e.what());
    }
    
    return false;
}

bool A05Ultrasonic::parseUARTData(const std::vector<uint8_t>& data, std::vector<float>& distances, std::vector<bool>& valid) {
    if (data.size() != 10) return false;
    
    if (data[0] != 0xFF) return false;
    
    uint8_t calculated_checksum = 0;
    for (int i = 0; i < 9; i++) {
        calculated_checksum += data[i];
    }
    calculated_checksum &= 0x00FF;
    
    if (calculated_checksum != data[9]) {
        ROS_WARN("Checksum mismatch in UART data");
        return false;
    }
    
    distances.resize(4);
    valid.resize(4, true);
    
    for (int i = 0; i < 4; i++) {
        uint16_t distance_mm = (data[1 + 2*i] << 8) | data[2 + 2*i];
        distances[i] = distance_mm / 1000.0f;
        valid[i] = (distance_mm >= 25 && distance_mm <= 4500);
    }
    
    return true;
}

bool A05Ultrasonic::readRS485Data(std::vector<float>& distances, std::vector<bool>& valid) {
    if (!connected_) return false;
    
    if (!sendRS485Command(0x01)) {
        return false;
    }
    
    std::vector<uint8_t> data;
    size_t bytes_read = serial_.read(data, 14);
    
    if (bytes_read == 14) {
        return parseRS485Data(data, distances, valid);
    }
    
    return false;
}

bool A05Ultrasonic::sendRS485Command(uint8_t command) {
    if (!connected_) return false;
    
    std::vector<uint8_t> cmd = {0x55, 0xAA, 0x01, command};
    uint8_t checksum = (0x55 + 0xAA + 0x01 + command) & 0x00FF;
    cmd.push_back(checksum);
    
    try {
        size_t bytes_written = serial_.write(cmd);
        return bytes_written == cmd.size();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Error writing to serial port: " << e.what());
        return false;
    }
}

bool A05Ultrasonic::parseRS485Data(const std::vector<uint8_t>& data, std::vector<float>& distances, std::vector<bool>& valid) {
    if (data.size() != 14) return false;
    
    if (data[0] != 0x55 || data[1] != 0xAA) return false;
    
    uint16_t sum = 0;
    for (int i = 0; i < 13; i++) {
        sum += data[i];
    }
    uint8_t calculated_checksum = sum & 0x00FF;
    
    if (calculated_checksum != data[13]) {
        ROS_WARN("Checksum mismatch in RS485 data");
        return false;
    }
    
    distances.resize(4);
    valid.resize(4, true);
    
    for (int i = 0; i < 4; i++) {
        uint16_t distance_mm = (data[4 + 2*i] << 8) | data[5 + 2*i];
        distances[i] = distance_mm / 1000.0f;
        valid[i] = (distance_mm >= 25 && distance_mm <= 4500);
    }
    
    return true;
}