#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>

class ImuMonitor {
public:
    ImuMonitor(ros::NodeHandle& nh) {
        // Initialize sums and counts to 0
        for (int i = 0; i < 6; ++i) {
            sums_[i] = 0.0;
            counts_[i] = 0;
        }

        // Subscribe to 6 IMU topics
        for (int i = 0; i < 6; ++i) {
            std::string topic = "/imu" + std::to_string(i) + "/data_raw";
            // Use boost::bind to pass the index to the callback
            subs_.push_back(nh.subscribe<sensor_msgs::Imu>(
                topic, 10, boost::bind(&ImuMonitor::imuCallback, this, _1, i)));
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg, int index) {
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;
        
        // Calculate magnitude: sqrt(x^2 + y^2 + z^2)
        double mag = std::sqrt(ax*ax + ay*ay + az*az);
        
        sums_[index] += mag;
        counts_[index]++;
    }

    void printStatus() {
        // Check if all sensors have at least 100 samples
        bool ready = true;
        for (int i = 0; i < 6; ++i) {
            if (counts_[i] < 100) {
                ready = false;
                break;
            }
        }

        if (ready) {
            // Use carriage return '\r' to overwrite the current line
            std::cout << "\r";
            std::cout << std::fixed << std::setprecision(5);
            
            for (int i = 0; i < 6; ++i) {
                double avg = sums_[i] / counts_[i];
                std::cout << "IMU" << i << ": " << std::setw(6) << avg << "  ";
                
                // Reset for next batch
                sums_[i] = 0.0;
                counts_[i] = 0;
            }
            // Flush to ensure output appears immediately
            std::cout << std::flush;
        }
    }

private:
    std::vector<ros::Subscriber> subs_;
    double sums_[6];
    int counts_[6];
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_magnitude_printer");
    ros::NodeHandle nh;
    
    ImuMonitor monitor(nh);

    // Update the display at 20Hz
    ros::Rate rate(10);
    
    ROS_INFO("Starting IMU Magnitude Printer...");
    
    while (ros::ok()) {
        ros::spinOnce();
        monitor.printStatus();
        rate.sleep();
    }
    
    // Print a newline at the end to clean up the terminal
    std::cout << std::endl;
    
    return 0;
}