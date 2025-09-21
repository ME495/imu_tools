#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__))))

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs_ext.msg import accelerometer
from registers import *
from mpu_6050 import MPU6050
import math
import numpy as np
import smbus
import threading
import time
import collections
from queue import Queue, Empty

# 五个MPU9250的配置
mpu_configs = [
    {"channel": 0, "address": MPU6050_ADDRESS_68},  # 第一个MPU9250
    {"channel": 1, "address": MPU6050_ADDRESS_68},  # 第二个MPU9250
    {"channel": 2, "address": MPU6050_ADDRESS_68},  # 第三个MPU9250
    {"channel": 3, "address": MPU6050_ADDRESS_68},  # 第四个MPU9250
    {"channel": 4, "address": MPU6050_ADDRESS_68},  # 第五个MPU9250
    {"channel": 5, "address": MPU6050_ADDRESS_68}   # 第六个MPU9250
]

aScale = [
    [0.992899, 1.002906, 0.986993],
    [0.997636, 1.005341, 0.968575],
    [0.991392, 0.995892, 0.986960],
    [0.863835, 0.849410, 1.091421],
    [1.001137, 0.992201, 0.976579],
    [1.002384, 1.000387, 0.977064]
]

aBias = [
    [ 0.126977,  0.759203, -1.740969],
    [-0.405112, -0.057797,  0.740100],
    [-0.366330, -0.083155,  0.307014],
    [13.820956, -50.927444, -12.073433],
    [-0.458899,  0.161197,  0.670736],
    [-0.376146,  0.243977,  1.232993]
]

gBias = [
    [ 0.019616, -0.034057, -0.035961],
    [-0.059039, -0.002755,  0.068250],
    [-0.017145,  0.035255,  0.127637],
    [ 0.517503,  9.401039,  0.119249],
    [-0.066121, -0.020660, -0.021138],
    [-0.061754,  0.004390, -0.010513]
]

class MultiMPU6050:
    def __init__(self):
        # 初始化I2C总线
        self.bus = smbus.SMBus(1)  # 使用树莓派的I2C-1
        self.pca9548a_address = 0x70  # PCA9548A的I2C地址
        
        # 存储MPU9250实例的列表
        self.mpus = []
        
        # I2C总线访问锁
        self.i2c_lock = threading.Lock()
        
        # 创建IMU数据队列和条件变量
        self.data_queues = [Queue(maxsize=10) for _ in range(len(mpu_configs))]
        self.data_ready = threading.Condition()
        self.new_data_available = [False] * len(mpu_configs)
        
        # 目标采样率设置
        self.target_rate = 125  # 目标帧率
        self.period = 1.0 / self.target_rate  # 周期时间(秒)1
        
        # 数据采集运行标志
        self.running = True
        
        # 帧率统计相关
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0
        
    def select_channel(self, channel):
        """选择PCA9548A的通道"""
        if 0 <= channel <= 7:
            self.bus.write_byte(self.pca9548a_address, 1 << channel)
        
    def initialize_mpus(self):
        """初始化所有MPU9250传感器"""
        with self.i2c_lock:
            for config in mpu_configs:
                # 选择对应的通道
                self.select_channel(config["channel"])
                time.sleep(0.001)  # 短暂等待通道切换
                
                try:
                    # 初始化MPU6050
                    mpu = MPU6050(
                        address_mpu_master=config["address"],
                        address_mpu_slave=None, 
                        bus=1,
                        gfs=GFS_1000,
                        afs=AFS_8G
                    )
                    
                    # 配置和校准传感器
                    mpu.configure()
                    # mpu.calibrateMPU6050()
                    # mpu.configure()
                    
                    self.mpus.append({
                        "channel": config["channel"],
                        "mpu": mpu
                    })
                    rospy.loginfo(f"MPU6050在通道{config['channel']}初始化成功，采样率设为{self.target_rate}Hz")
                    
                except Exception as e:
                    rospy.logerr(f"初始化通道{config['channel']}的MPU6050失败: {e}")
                    import traceback
                    traceback.print_exc()
        
        # 启动单一数据采集线程
        self.data_thread = threading.Thread(target=self.data_collection_thread)
        self.data_thread.daemon = True
        self.data_thread.start()

    def data_collection_thread(self):
        """
        【优化版】单一线程处理所有传感器数据采集。
        采用精确的周期定时策略，防止误差累积。
        """
        # 使用 time.perf_counter() 以获得更高精度的计时
        next_cycle_time = time.perf_counter()
        
        while self.running and not rospy.is_shutdown():
            data_collected = False
            
            with self.i2c_lock:
                for i, mpu_data in enumerate(self.mpus):
                    try:
                        # 选择对应的通道
                        self.select_channel(mpu_data["channel"])
                        
                        # 小延时确保通道切换完成
                        # time.sleep(0.0001)
                        
                        mpu : MPU6050 = mpu_data["mpu"]
                        
                        # 读取传感器数据
                        accel = mpu.readAccelerometerMaster()
                        gyro = mpu.readGyroscopeMaster()
                        
                        # 将数据放入队列
                        imu_data = {
                            'accel': accel,
                            'gyro': gyro
                        }
                        
                        # 非阻塞方式加入队列，如果队列满则丢弃旧数据
                        if self.data_queues[i].full():
                            try:
                                self.data_queues[i].get_nowait()  # 移除最旧的数据
                            except Empty:
                                pass
                        
                        self.data_queues[i].put_nowait(imu_data)
                        data_collected = True
                        
                        # 标记数据就绪
                        with self.data_ready:
                            self.new_data_available[i] = True
                            
                    except Exception as e:
                        rospy.logerr(f"读取通道{mpu_data['channel']}的传感器数据时出错: {e}")
            
            # 如果有新数据，通知所有等待的线程
            if data_collected:
                with self.data_ready:
                    self.data_ready.notify_all()
            '''
            # 统计帧率
            self.frame_count += 1
            current_time = time.time()
            time_diff = current_time - self.last_time
            
            # 每秒计算一次帧率
            if time_diff >= 1.0:
                self.fps = self.frame_count / time_diff
                rospy.loginfo(f"传感器数据采集帧率: {self.fps:.2f} Hz")
                self.frame_count = 0
                self.last_time = current_time
            '''
            
            # --- 精确定时 ---
            # 计算下一个周期的目标唤醒时间
            next_cycle_time += self.period
            
            # 计算需要睡眠的时间
            sleep_duration = next_cycle_time - time.perf_counter()

            if sleep_duration > 0:
                time.sleep(sleep_duration)
            else:
                # 如果已经超时，立即进入下一个循环，并记录警告
                rospy.logwarn(f"采集周期耗时过长，错过目标时间点 {-sleep_duration*1000:.2f}ms")
                # 如果系统持续超时，可能需要重新同步目标时间，防止误差越拉越大
                if sleep_duration < -self.period:
                    next_cycle_time = time.perf_counter() + self.period
    
    def get_latest_data(self, imu_index):
        """获取指定IMU的最新数据"""
        try:
            return self.data_queues[imu_index].get_nowait()
        except Empty:
            return None
    
    def wait_for_data(self, timeout=None):
        """等待新数据可用，timeout=None表示无限等待直到有数据"""
        with self.data_ready:
            # 如果没有新数据，阻塞等待
            if not any(self.new_data_available):
                self.data_ready.wait(timeout)
            
            # 返回有新数据的IMU索引
            result = [i for i, available in enumerate(self.new_data_available) if available]
            # 重置标志
            self.new_data_available = [False] * len(self.new_data_available)
            return result
    
    def shutdown(self):
        """停止数据采集线程"""
        self.running = False
        # 唤醒所有可能被阻塞的线程
        with self.data_ready:
            self.data_ready.notify_all()
            
        if hasattr(self, 'data_thread') and self.data_thread.is_alive():
            self.data_thread.join(timeout=1.0)
        

def main():
    # 初始化ROS节点
    rospy.init_node('multi_mpu6050_node')
    
    # 创建发布器列表
    publishers = []
    for i in range(len(mpu_configs)):  # 六个传感器
        pubs = {
            'imu': rospy.Publisher(f'imu{i}/data_raw', Imu, queue_size=100),
            'ext_acc': rospy.Publisher(f'imu{i}/accelerometer', accelerometer, queue_size=100)
        }
        publishers.append(pubs)
    
    # 初始化多传感器管理器
    multi_mpu = MultiMPU6050()
    multi_mpu.initialize_mpus()
    
    # 定义单位转换常量
    DEG_TO_RAD = math.pi / 180.0
    G_TO_M_S2 = 9.81
    
    # 初始化IMU消息
    imu_msgs = []
    ext_acc_msgs = []
    for i in range(len(mpu_configs)):
        imu_msg = Imu()
        imu_msg.header.frame_id = f"imu{i}_link"
        imu_msg.orientation_covariance = [-1, 0, 0, 0, -1, 0, 0, 0, -1]
        imu_msg.angular_velocity_covariance = [-1, 0, 0, 0, -1, 0, 0, 0, -1]
        imu_msg.linear_acceleration_covariance = [-1, 0, 0, 0, -1, 0, 0, 0, -1]
        imu_msgs.append(imu_msg)
        
        ext_acc_msg = accelerometer()
        ext_acc_msgs.append(ext_acc_msg)

    rospy.loginfo(f"开始以{multi_mpu.target_rate}Hz的速率发布多传感器数据...")

    # 注册关闭回调函数
    rospy.on_shutdown(multi_mpu.shutdown)
    
    # 主循环 - ROS消息发布帧率统计
    frame_count = 0
    last_time = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        try:
            # 无限期等待新数据可用（完全阻塞）
            ready_imu_indices = multi_mpu.wait_for_data(timeout=None)
            
            # 获取当前时间，作为这一批数据的时间戳
            now = rospy.Time.now()
            
            # 处理所有有新数据的IMU
            for i in ready_imu_indices:
                # 获取最新数据
                imu_data = multi_mpu.get_latest_data(i)
                
                if imu_data:
                    # 更新时间戳
                    imu_msgs[i].header.stamp = now
                    
                    # 填充IMU消息
                    imu_msgs[i].linear_acceleration.x = imu_data['accel'][0] * G_TO_M_S2 * aScale[i][0] + aBias[i][0]
                    imu_msgs[i].linear_acceleration.y = imu_data['accel'][1] * G_TO_M_S2 * aScale[i][1] + aBias[i][1]
                    imu_msgs[i].linear_acceleration.z = imu_data['accel'][2] * G_TO_M_S2 * aScale[i][2] + aBias[i][2]
                    
                    imu_msgs[i].angular_velocity.x = imu_data['gyro'][0] * DEG_TO_RAD - gBias[i][0]
                    imu_msgs[i].angular_velocity.y = imu_data['gyro'][1] * DEG_TO_RAD - gBias[i][1]
                    imu_msgs[i].angular_velocity.z = imu_data['gyro'][2] * DEG_TO_RAD - gBias[i][2]
                    
                    # 填充扩展加速度消息
                    ext_acc_msgs[i].x = imu_msgs[i].linear_acceleration.x
                    ext_acc_msgs[i].y = imu_msgs[i].linear_acceleration.y
                    ext_acc_msgs[i].z = imu_msgs[i].linear_acceleration.z
                    
                    # 发布消息
                    publishers[i]['imu'].publish(imu_msgs[i])
                    publishers[i]['ext_acc'].publish(ext_acc_msgs[i])
            '''
            # 统计发布帧率
            frame_count += 1
            current_time = rospy.Time.now().to_sec()
            elapsed = current_time - last_time
            
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                rospy.loginfo(f"ROS消息发布帧率: {fps:.2f} Hz")
                frame_count = 0
                last_time = current_time
            '''
                
        except Exception as e:
            rospy.logerr(f"主循环处理数据错误: {e}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 
