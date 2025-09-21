#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu

class GyroCalibrator:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('gyro_calibrator', anonymous=True)

        # 从参数服务器获取配置
        self.imu_topic = rospy.get_param('~imu_topic', '/imu/data_raw')
        self.num_samples = rospy.get_param('~num_samples', 500)

        # 用于存储陀螺仪数据的列表
        self.samples = []
        self.calibration_done = False

        # 创建订阅者
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

        rospy.loginfo("Gyro Calibrator Node Started.")
        rospy.loginfo("Please keep the IMU completely still.")
        rospy.loginfo(f"Collecting {self.num_samples} samples from topic {self.imu_topic}...")

    def imu_callback(self, msg):
        # 如果标定未完成，则收集数据
        if not self.calibration_done:
            # 从消息中提取角速度数据
            angular_velocity = [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]
            self.samples.append(angular_velocity)

            # 检查是否已收集到足够样本
            if len(self.samples) >= self.num_samples:
                self.perform_calibration()

    def perform_calibration(self):
        self.calibration_done = True
        self.imu_sub.unregister() # 停止订阅，节省资源

        rospy.loginfo("Sample collection complete. Calculating bias...")

        # 将样本列表转换为Numpy数组，方便计算
        samples_array = np.array(self.samples)
        
        # 沿列（axis=0）计算平均值
        bias = np.mean(samples_array, axis=0)

        # 打印结果
        rospy.loginfo("-----------------------------------------------------")
        rospy.loginfo("Gyroscope Bias Calculation Complete!")
        rospy.loginfo(f"Bias X: {bias[0]:.6f} rad/s")
        rospy.loginfo(f"Bias Y: {bias[1]:.6f} rad/s")
        rospy.loginfo(f"Bias Z: {bias[2]:.6f} rad/s")
        rospy.loginfo("-----------------------------------------------------")
        rospy.loginfo("You can now copy these values into your main application or configuration file.")
        rospy.loginfo("Shutting down node.")
        
        # 关闭节点
        rospy.signal_shutdown("Calibration finished.")

if __name__ == '__main__':
    try:
        GyroCalibrator()
        rospy.spin() # 保持节点运行直到标定完成并关闭
    except rospy.ROSInterruptException:
        pass