#include "a05_ultrasonic/a05_ultrasonic.h"
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <string>
#include <vector>

class UltrasonicNode {
public:
    UltrasonicNode() : nh_("~") {
        // Initialize parameters
        std::string port;
        int baudrate;
        bool simulate;
        double publish_rate;

        nh_.param<std::string>("port", port, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate, 38400);
        nh_.param<bool>("simulate", simulate, false);
        nh_.param<double>("publish_rate", publish_rate, 10.0);

        // Initialize sensor
        sensor_ = std::make_unique<A05Ultrasonic>(port, baudrate);
        sensor_->setSimulationMode(simulate);

        if (!simulate && !sensor_->connect()) {
            ROS_ERROR("Failed to connect to A05 ultrasonic sensor");
            ros::shutdown();
            return;
        }

        // Initialize publishers for each probe
        for (int i = 0; i < 4; i++) {
            std::string topic_name = "ultrasonic/probe_" + std::to_string(i+1);
            pubs_.push_back(nh_.advertise<sensor_msgs::Range>(topic_name, 10));
        }

        // Start timer
        timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate), &UltrasonicNode::timerCallback, this);

        ROS_INFO("A05 Ultrasonic Node initialized and ready");
    }

    void timerCallback(const ros::TimerEvent& event) {
        std::vector<float> distances;
        std::vector<bool> valid;

        if (sensor_->readData(distances, valid)) {
            for (size_t i = 0; i < 4 && i < distances.size(); i++) {
                sensor_msgs::Range range_msg;
                range_msg.header.stamp = ros::Time::now();
                range_msg.header.frame_id = "ultrasonic_" + std::to_string(i+1);
                
                // Configure range message
                range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
                range_msg.field_of_view = 1.0472; // 60 degrees in radians
                range_msg.min_range = 0.25;       // 25cm
                range_msg.max_range = 4.5;        // 450cm
                range_msg.range = valid[i] ? distances[i] : std::numeric_limits<float>::infinity();

                pubs_[i].publish(range_msg);

                // Print to console
                if (valid[i]) {
                    //ROS_INFO("Probe %zu: %.3f meters", i+1, distances[i]);
                } else {
                    ROS_WARN("Probe %zu: Invalid reading", i+1);
                }
            }
        } else {
            ROS_ERROR("Failed to read data from sensor");
        }
    }

private:
    ros::NodeHandle nh_;
    std::unique_ptr<A05Ultrasonic> sensor_;
    std::vector<ros::Publisher> pubs_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "a05_ultrasonic_node");
    UltrasonicNode node;
    ros::spin();
    return 0;
}