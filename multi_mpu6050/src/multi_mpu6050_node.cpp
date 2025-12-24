#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs_ext/accelerometer.h>
#include <vector>
#include <string>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>

// MPU6050 Registers
#define MPU6050_ADDR 0x68
#define PCA9548A_ADDR 0x70

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B

// Configuration Values
#define DLPF_CFG_44HZ    0x03
#define SMPLRT_DIV_100HZ 0x09
#define GFS_2000DPS      0x18 // (3 << 3)
#define AFS_16G          0x18 // (3 << 3)

// Scales
const double ACCEL_SCALE = 16.0 / 32768.0;
const double GYRO_SCALE = 2000.0 / 32768.0;
const double G_TO_M_S2 = 9.80665;
const double DEG_TO_RAD = M_PI / 180.0;

struct MpuConfig {
    int index;   // Index in the publisher list
    int channel; // Physical channel on PCA9548A
};

// Raw data structure for one sensor
struct SensorRawData {
    int index;
    int channel;
    uint8_t data[14];
    ros::Time timestamp;
};

// Frame structure containing data for all sensors
struct FrameData {
    std::vector<SensorRawData> sensors;
};

// Calibration Data
const double aScale[6][3] = {
    {0.9968333815, 1.0047237210, 0.9800578609}, // Channel 0
    {1.0040362671, 0.9938512167, 0.9870425091}, // Channel 1
    {0.9937053441, 1.0037685366, 0.9903839457}, // Channel 2
    {0.9990213834, 1.0037774456, 0.9772431261}, // Channel 3
    {0.9994529031, 0.9971132517, 0.9892505038}, // Channel 4
    {0.9893060013, 0.9949944391, 0.9841812878}  // Channel 5
};

const double aBias[6][3] = {
    {-0.5011657661, 0.1296881335, 0.6807852740}, // Channel 0
    {-0.8867132237,-5.9778617655, 0.7051673104}, // Channel 1
    {-0.2338010518, 0.0036652022,-0.1369682981}, // Channel 2
    {-0.3154475471, 0.3312055625,-4.2380675780}, // Channel 3
    {-0.4088827418,-0.0722272956,-0.0757277605}, // Channel 4
    { 5.4408319901,-3.3678231232, 0.4706781447}  // Channel 5
};

// const double aScale[6][3] = {
//     {1., 1., 1.}, // Channel 0
//     {1., 1., 1.}, // Channel 1
//     {1., 1., 1.}, // Channel 2
//     {1., 1., 1.}, // Channel 3
//     {1., 1., 1.}, // Channel 4
//     {1., 1., 1.}  // Channel 5
// };

// const double aBias[6][3] = {
//     { 0., 0., 0.}, // Channel 0
//     { 0., 0., 0.}, // Channel 1
//     { 0., 0., 0.}, // Channel 2
//     { 0., 0., 0.}, // Channel 3
//     { 0., 0., 0.}, // Channel 4
//     { 0., 0., 0.}  // Channel 5
// };

class MultiMpuNode {
public:
    MultiMpuNode() : nh_("~"), running_(true) {
        // Initialize I2C
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            ROS_FATAL("Failed to open /dev/i2c-1");
            ros::shutdown();
            return;
        }

        // Define Sensors
        configs_.push_back({0, 0}); // Index 0, Channel 0
        configs_.push_back({1, 1}); // Index 1, Channel 1
        configs_.push_back({2, 2}); // Index 2, Channel 2
        configs_.push_back({3, 3}); // Index 3, Channel 3
        configs_.push_back({4, 4}); // Index 4, Channel 4
        configs_.push_back({5, 5}); // Index 5, Channel 5

        // Initialize Publishers and Sensors
        for (const auto& config : configs_) {
            std::string topic_prefix = "imu" + std::to_string(config.channel);
            imu_pubs_.push_back(nh_.advertise<sensor_msgs::Imu>("/" + topic_prefix + "/data_raw", 100));
            acc_pubs_.push_back(nh_.advertise<sensor_msgs_ext::accelerometer>("/" + topic_prefix + "/accelerometer", 100));
            
            initMpu(config.channel);
        }

        // Start Acquisition Thread
        acquire_thread_ = std::thread(&MultiMpuNode::acquireLoop, this);

        ROS_INFO("Multi MPU6050 Node Initialized (Multithreaded C++)");
    }

    ~MultiMpuNode() {
        running_ = false;
        if (acquire_thread_.joinable()) {
            acquire_thread_.join();
        }
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
        }
    }

    // Main thread acts as the Publishing Thread
    void run() {
        while (ros::ok()) {
            FrameData frame;
            bool has_data = false;

            // Wait for data
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                // 使用 wait_for 设置 100ms 超时，防止 Ctrl+C 时死锁
                if (!queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { return !queue_.empty() || !running_; })) {
                    // 超时醒来，检查是否需要退出
                    if (!ros::ok()) break;
                    continue;
                }
                
                if (!running_ && queue_.empty()) break;

                if (!queue_.empty()) {
                    frame = queue_.front();
                    queue_.pop_front();
                    has_data = true;
                }
            }

            if (has_data) {
                processAndPublish(frame);
            }
            
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    int i2c_fd_;
    std::vector<MpuConfig> configs_;
    std::vector<ros::Publisher> imu_pubs_;
    std::vector<ros::Publisher> acc_pubs_;
    
    // Dynamic Gyro Bias (Calibrated at startup)
    // Indexed by channel [0-5]
    double gyro_bias_[6][3] = {0};

    // Threading
    std::thread acquire_thread_;
    std::atomic<bool> running_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<FrameData> queue_;

    bool selectChannel(int channel) {
        if (ioctl(i2c_fd_, I2C_SLAVE, PCA9548A_ADDR) < 0) {
            ROS_WARN_THROTTLE(1, "Failed to select PCA9548A address: %s", strerror(errno));
            return false;
        }
        uint8_t data = 1 << channel;
        if (write(i2c_fd_, &data, 1) != 1) {
            ROS_WARN_THROTTLE(1, "Failed to switch to channel %d: %s", channel, strerror(errno));
            return false;
        }
        return true;
    }

    void writeReg(uint8_t reg, uint8_t val) {
        uint8_t data[2] = {reg, val};
        if (write(i2c_fd_, data, 2) != 2) {
            ROS_ERROR_THROTTLE(1, "Failed to write register 0x%02X: %s", reg, strerror(errno));
        }
    }

    void initMpu(int channel) {
        if (!selectChannel(channel)) {
            ROS_FATAL("Failed to select channel %d during initialization.", channel);
            ros::shutdown();
            exit(1);
        }
        if (ioctl(i2c_fd_, I2C_SLAVE, MPU6050_ADDR) < 0) {
            ROS_FATAL("Failed to select MPU6050 address on channel %d: %s", channel, strerror(errno));
            ros::shutdown();
            exit(1);
        }

        writeReg(REG_PWR_MGMT_1, 0x80); // Reset
        usleep(100000);
        writeReg(REG_PWR_MGMT_1, 0x00); // Wake up
        usleep(100000);
        writeReg(REG_CONFIG, DLPF_CFG_44HZ);
        writeReg(REG_SMPLRT_DIV, SMPLRT_DIV_100HZ);
        writeReg(REG_GYRO_CONFIG, GFS_2000DPS);
        writeReg(REG_ACCEL_CONFIG, AFS_16G);

        ROS_INFO("Initialized MPU6050 on channel %d", channel);
    }

    void acquireLoop() {
        // --- Calibration Phase ---
        ROS_INFO("Starting Gyroscope Calibration (100 frames)... Please keep IMUs stationary.");
        int32_t gyro_sum[6][3] = {0}; // Accumulator for each channel
        int samples = 100;

        for (int i = 0; i < samples; ++i) {
            for (const auto& config : configs_) {
                if (!selectChannel(config.channel)) {
                    ROS_WARN_THROTTLE(1, "Calibration: Failed to select channel %d. Skipping.", config.channel);
                    continue;
                }
                if (ioctl(i2c_fd_, I2C_SLAVE, MPU6050_ADDR) < 0) {
                    ROS_WARN_THROTTLE(1, "Calibration: Failed to select MPU on channel %d: %s", config.channel, strerror(errno));
                    continue;
                }

                uint8_t reg = REG_ACCEL_XOUT_H;
                if (write(i2c_fd_, &reg, 1) != 1) {
                    ROS_WARN_THROTTLE(1, "Calibration: Failed to write reg on channel %d: %s", config.channel, strerror(errno));
                    continue;
                }

                uint8_t buf[14];
                if (read(i2c_fd_, buf, 14) == 14) {
                    // Parse Gyro (Bytes 8-13)
                    int16_t gx = (buf[8] << 8) | buf[9];
                    int16_t gy = (buf[10] << 8) | buf[11];
                    int16_t gz = (buf[12] << 8) | buf[13];
                    
                    gyro_sum[config.channel][0] += gx;
                    gyro_sum[config.channel][1] += gy;
                    gyro_sum[config.channel][2] += gz;
                }
            }
            usleep(10000); // 10ms interval
        }

        // Calculate Average Bias
        for (int ch = 0; ch < 6; ++ch) {
            gyro_bias_[ch][0] = (double)gyro_sum[ch][0] / samples * GYRO_SCALE;
            gyro_bias_[ch][1] = (double)gyro_sum[ch][1] / samples * GYRO_SCALE;
            gyro_bias_[ch][2] = (double)gyro_sum[ch][2] / samples * GYRO_SCALE;
        }
        ROS_INFO("Calibration Complete. Gyro Biases updated.");

        // --- Main Acquisition Loop ---
        ros::Rate rate(100); // 100Hz
        
        while (running_ && ros::ok()) {
            ros::Time start_time = ros::Time::now();
            
            FrameData frame;
            frame.sensors.reserve(configs_.size());

            for (const auto& config : configs_) {
                // 1. Switch Channel
                if (!selectChannel(config.channel)) {
                    ROS_WARN_THROTTLE(1, "Failed to switch to channel %d. Skipping.", config.channel);
                    continue;
                }
                
                // 2. Select MPU
                if (ioctl(i2c_fd_, I2C_SLAVE, MPU6050_ADDR) < 0) {
                    ROS_WARN_THROTTLE(1, "Failed to select MPU on channel %d: %s", config.channel, strerror(errno));
                    continue;
                }

                // 3. Timestamp
                ros::Time now = ros::Time::now();

                // 4. Read Raw Data
                uint8_t reg = REG_ACCEL_XOUT_H;
                if (write(i2c_fd_, &reg, 1) != 1) {
                    ROS_WARN_THROTTLE(1, "Failed to write reg address on channel %d: %s", config.channel, strerror(errno));
                    continue;
                }
                
                SensorRawData sensor_data;
                sensor_data.index = config.index;
                sensor_data.channel = config.channel;
                sensor_data.timestamp = now;

                if (read(i2c_fd_, sensor_data.data, 14) == 14) {
                    frame.sensors.push_back(sensor_data);
                } else {
                    ROS_WARN_THROTTLE(1, "Failed to read data on channel %d: %s", config.channel, strerror(errno));
                }
            }

            // Push to queue
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                // // Drop old data if queue is full (keep latest)
                // if (queue_.size() > 5) {
                //     queue_.pop_front();
                // }
                queue_.push_back(frame);
            }
            queue_cv_.notify_one();

            // Timing check
            ros::Time end_time = ros::Time::now();
            double duration = (end_time - start_time).toSec();
            if (duration > 0.01) {
                ROS_WARN("Acquisition cycle too long: %.2f ms", duration * 1000.0);
            }

            rate.sleep();
        }
        
        // 退出循环后，确保通知主线程，防止主线程死锁
        running_ = false;
        queue_cv_.notify_all();
    }

    void processAndPublish(const FrameData& frame) {
        for (const auto& sensor : frame.sensors) {
            // Parse
            int16_t ax_raw = (sensor.data[0] << 8) | sensor.data[1];
            int16_t ay_raw = (sensor.data[2] << 8) | sensor.data[3];
            int16_t az_raw = (sensor.data[4] << 8) | sensor.data[5];
            int16_t gx_raw = (sensor.data[8] << 8) | sensor.data[9];
            int16_t gy_raw = (sensor.data[10] << 8) | sensor.data[11];
            int16_t gz_raw = (sensor.data[12] << 8) | sensor.data[13];

            // Convert
            int ch = sensor.channel;
            // Accel: Apply Scale then Bias (as per python script logic: raw * scale * scale_ext + bias_ext)
            // Note: Python script logic was: acc_x = (raw * ares) - abias_internal. 
            // But we don't do internal calibration here, so abias_internal is 0.
            // Then msg.x = acc_x * G * aScale + aBias.
            double ax = ax_raw * ACCEL_SCALE * aScale[ch][0] * G_TO_M_S2 + aBias[ch][0];
            double ay = ay_raw * ACCEL_SCALE * aScale[ch][1] * G_TO_M_S2 + aBias[ch][1];
            double az = az_raw * ACCEL_SCALE * aScale[ch][2] * G_TO_M_S2 + aBias[ch][2];

            // Gyro: Apply Scale then Dynamic Bias
            double gx = (gx_raw * GYRO_SCALE - gyro_bias_[ch][0]) * DEG_TO_RAD;
            double gy = (gy_raw * GYRO_SCALE - gyro_bias_[ch][1]) * DEG_TO_RAD;
            double gz = (gz_raw * GYRO_SCALE - gyro_bias_[ch][2]) * DEG_TO_RAD;

            // Publish
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = sensor.timestamp;
            imu_msg.header.frame_id = "imu" + std::to_string(sensor.channel) + "_link";
            
            imu_msg.linear_acceleration.x = ax;
            imu_msg.linear_acceleration.y = ay;
            imu_msg.linear_acceleration.z = az;
            
            imu_msg.angular_velocity.x = gx;
            imu_msg.angular_velocity.y = gy;
            imu_msg.angular_velocity.z = gz;

            imu_msg.orientation_covariance[0] = -1;
            imu_msg.angular_velocity_covariance[0] = -1;
            imu_msg.linear_acceleration_covariance[0] = -1;

            imu_pubs_[sensor.index].publish(imu_msg);

            sensor_msgs_ext::accelerometer acc_msg;
            acc_msg.x = ax;
            acc_msg.y = ay;
            acc_msg.z = az;
            acc_pubs_[sensor.index].publish(acc_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_mpu6050_node_cpp");
    MultiMpuNode node;
    node.run();
    return 0;
}