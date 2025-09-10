IMU Tools for ROS / ROS 2
=========================

一个包含 IMU 融合滤波与可视化工具的集合，支持 ROS1 与 ROS2。主要组件：
- imu_filter_madgwick：Madgwick 融合滤波器
- imu_complementary_filter：互补滤波器（四元数）
- rviz_imu_plugin：RViz 插件显示 sensor_msgs/Imu
- multi_mpu9250（ROS1）：通过 I2C 多路复用器读取多枚 MPU9250 并发布 /imuN/data_raw，且可接入互补滤波

目录
----
- 快速概览
- 安装
  - 通过二进制安装
  - 从源码安装（ROS1）
  - 从源码安装（ROS2）
- multi_mpu9250 使用指南（ROS1）
  - 功能概述
  - 硬件与系统依赖
  - 安装依赖（示例）
  - 硬件连接要点
  - 构建与安装
  - 启动方式
  - 话题与坐标系
  - 参数与可定制化
  - 验证与可视化
  - 常见问题与排查
  - 包内文件参考
- Docker 部署到树莓派（Raspberry Pi）
- 开发辅助：pre-commit 检查
- 许可协议
- 参考资料

快速概览
--------
本仓库包含：
- imu_filter_madgwick：融合角速度/加速度/（可选）磁力计为姿态，基于 [1]
- imu_complementary_filter：基于互补融合的四元数姿态估计，基于 [2]
- rviz_imu_plugin：RViz 插件，显示 sensor_msgs/Imu
- multi_mpu9250（ROS1）：多 MPU9250 采集与（可选）每路互补滤波流水线

参考：
[1]: https://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
[2]: https://www.mdpi.com/1424-8220/15/8/19302

安装
----
通过二进制安装
- 本仓库已发布到当前的 ROS1 和 ROS2 发行版，可直接安装：
  sudo apt-get install ros-<YOUR_ROSDISTO>-imu-tools

从源码安装（ROS1）
- 创建并初始化 catkin 工作空间（例如 ~/catkin_ws/），并 source devel/setup.bash：
  参考 https://wiki.ros.org/catkin/Tutorials/create_a_workspace
- 安装 git：
  sudo apt-get install git
- 克隆本仓库到工作空间 src 目录，使用与你发行版匹配的分支（如 melodic、noetic 等）：
  git clone -b <YOUR_ROSDISTO> https://github.com/CCNYRoboticsLab/imu_tools.git
- 使用 rosdep 安装依赖：
  rosdep install --from-paths src --ignore-src -r -y
- 编译：
  cd ~/catkin_ws
  catkin_make

从源码安装（ROS2）
- 按 ROS2 官方流程创建工作区（colcon），将示例仓库替换为本仓库对应分支：
  参考 https://docs.ros.org/en/rolling/Tutorials/Workspace/Creating-A-Workspace.html
- 克隆：
  git clone -b <YOUR_ROSDISTO> https://github.com/CCNYRoboticsLab/imu_tools.git
- 安装依赖与构建方式请按所用 ROS2 发行版与系统环境调整。

multi_mpu9250 使用指南（ROS1）
------------------------------
功能概述
- 节点：multi_mpu9250_node
- 预置 IMU 数量：6（通道 0..5，可扩展）
- 每路发布话题：/imu{i}/data_raw（sensor_msgs/Imu）
  - 仅填充 linear_acceleration 与 angular_velocity，orientation 未估计（covariance = [-1 ...]）
- 可选：接入 imu_complementary_filter 输出 /imu{i}/data（姿态）
- 目标频率：≈ 100 Hz（单线程轮询采样，时间同步至采样时刻）

硬件与系统依赖
- 硬件
  - 树莓派或其他带 I2C 的 Linux SBC
  - I2C 多路复用器 PCA9548A（默认地址 0x70）
  - 多个 MPU9250（默认地址 0x68）
  - 可靠的 3.3V/5V 供电与公共地
- 软件
  - ROS1（如 Melodic/Noetic）
  - Python 2/3 + rospy
  - smbus/python-smbus
  - 第三方库 mpu9250_jmdev（Python）
  - 本仓库的 imu_complementary_filter（可选）
  - rviz_imu_plugin（可视化，可选）

安装依赖（示例，Ubuntu/ROS1）
- 系统包：
  sudo apt-get update
  sudo apt-get install python-smbus i2c-tools
- Python 包（按环境选择 pip 或 pip3）：
  pip3 install mpu9250-jmdev
- 启用 I2C（树莓派）：通过 raspi-config 或编辑 /boot 配置
- I2C 探测：
  sudo i2cdetect -y 1
  应看到 PCA9548A（0x70）及各通道下的 MPU（通常 0x68）

硬件连接要点
- 主控 SDA/SCL → PCA9548A SDA/SCL
- 每个 MPU9250 → PCA9548A 某下行通道（0..5）
- 供电与地线可靠、3.3V 电平兼容
- 多个 IMU 若地址相同（0x68），需通过多路复用器分离通道（当前实现基于此）

构建与安装
- 将本仓库放入 catkin 工作空间：~/catkin_ws/src/imu_tools
- 解析依赖并编译：
  cd ~/catkin_ws
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  source devel/setup.bash
- 安装后可执行脚本名：multi_mpu9250.py（见包内 CMakeLists）

启动方式
1) 使用提供的 launch（原始数据 + 每路互补滤波，共 6 路）：
  roslaunch multi_mpu9250 multi_mpu9250.launch
  该 launch 将：
  - 启动 multi_mpu9250_node
  - 为命名空间 imu0..imu5 启动 imu_complementary_filter/complementary_filter_node
    - 输入：/imuN/data_raw
    - 输出：/imuN/data
2) 直接运行节点（仅发布原始数据）：
  rosrun multi_mpu9250 multi_mpu9250.py

话题与坐标系
- 原始话题：/imuN/data_raw（N=0..5）
- 融合话题：/imuN/data（由互补滤波输出）
- frame_id：默认 imuN_link
- 加速度单位：m/s^2（由 g=9.81 转换）
- 角速度单位：rad/s（由 deg/s 转换）
- orientation 未估计（covariance = [-1 ...]）

参数与可定制化
- 传感器通道与地址定义： [multi_mpu9250/scripts/multi_mpu9250.py](multi_mpu9250/scripts/multi_mpu9250.py:16)
- PCA9548A 地址（默认 0x70）： [multi_mpu9250/scripts/multi_mpu9250.py](multi_mpu9250/scripts/multi_mpu9250.py:30)
- 目标采样率（默认 100 Hz）： [multi_mpu9250/scripts/multi_mpu9250.py](multi_mpu9250/scripts/multi_mpu9250.py:44)
- 量程/滤波配置（GFS_1000、AFS_8G 等，初始化处）： [multi_mpu9250/scripts/multi_mpu9250.py](multi_mpu9250/scripts/multi_mpu9250.py:75)
- 互补滤波器参数（自适应、磁力计、增益等，launch 中可改）： [multi_mpu9250/launch/multi_mpu9250.launch](multi_mpu9250/launch/multi_mpu9250.launch:7)

验证与可视化
- rostopic：
  rostopic list | grep imu
  rostopic echo /imu0/data_raw
  rostopic echo /imu0/data
- RViz（建议安装 rviz_imu_plugin）：
  rviz
  在 “IMU” 显示中选择 /imuN/data 或 /imuN/data_raw
- 录包/回放：
  rosbag record /imu0/data_raw /imu1/data_raw ...
  rosbag play your_record.bag

常见问题与排查
- 无话题
  - 确认已 source：source ~/catkin_ws/devel/setup.bash
  - 启动并检查日志：roslaunch multi_mpu9250 multi_mpu9250.launch
- I2C 读写错误（初始化/读取失败）
  - 检查 /dev/i2c-1 与探测：sudo i2cdetect -y 1
  - 检查 PCA9548A 地址（默认 0x70）
  - 供电/地线、SDA/SCL 连接、上拉电阻
- 多传感器通道冲突
  - 读取前切换至正确通道（代码已实现）
  - 同地址 IMU 必须不同通道
- 发布频率不稳
  - I2C 负载高或 Python 调度延迟，可降目标频率或减少 IMU 数量
- 依赖缺失（ModuleNotFoundError: mpu9250_jmdev）
  - pip/pip3 安装并确保与 ROS Python 版本一致

包内文件参考
- 启动： [multi_mpu9250/launch/multi_mpu9250.launch](multi_mpu9250/launch/multi_mpu9250.launch:1)
- 脚本： [multi_mpu9250/scripts/multi_mpu9250.py](multi_mpu9250/scripts/multi_mpu9250.py:1)
- 包描述： [multi_mpu9250/package.xml](multi_mpu9250/package.xml:1)
- 构建脚本： [multi_mpu9250/CMakeLists.txt](multi_mpu9250/CMakeLists.txt:1)

Docker 部署到树莓派（Raspberry Pi）
--------------------------------
以下示例展示在树莓派上通过 Docker 运行（包含 multi_mpu9250 与可选可视化/滤波链）。
启动容器：
sudo docker run -itd \
  --privileged \
  --network host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=10.0.0.201:0.0 \
  -v /home/imu:/home/imu \
  ME495/imu:0.4 \
  /bin/bash

参数说明
- --privileged 与 --network host：访问 I2C/GPIO 等硬件并复用主机网络（便于 ROS 通信）
- X11 显示转发：
  -v /tmp/.X11-unix:/tmp/.X11-unix 与 -e DISPLAY=10.0.0.201:0.0 用于容器内运行 RViz
  - 若在本机 Linux 显示，宿主机执行：
    xhost +local:root
    export DISPLAY=:0
    并在 docker run 使用 -e DISPLAY=:0
- 数据卷挂载：-v /home/imu:/home/imu 便于保存 bag/配置/日志
- I2C 设备：建议 --privileged，如仍失败可加 --device /dev/i2c-1:/dev/i2c-1；宿主机需启用 I2C

进入容器与运行
- 进入：
  docker exec -it <容器ID或名称> /bin/bash
- 启动 multi_mpu9250：
  roslaunch multi_mpu9250 multi_mpu9250.launch
- 仅发布原始数据：
  rosrun multi_mpu9250 multi_mpu9250.py

容器内验证
- 检查 I2C：ls /dev/i2c-1 && sudo i2cdetect -y 1
- 检查话题：rostopic list | grep imu
- 可视化：rviz（确保 DISPLAY 映射正确）

开发辅助：pre-commit 检查
-------------------------
本仓库在 CI 中使用 pre-commit。建议本地启用以保持一致格式。
- 安装：
  pip3 install --user pre-commit
- 手动运行：
  pre-commit run -a
- 安装 git hooks（提交前自动运行）：
  pre-commit install

许可协议
--------
- imu_filter_madgwick：GPL（沿用原实现）
- imu_complementary_filter：BSD
- rviz_imu_plugin：BSD

参考资料
--------
- 本仓库各节点、话题与参数详见 ROS Wiki：
  https://wiki.ros.org/imu_tools
- 论文与算法来源：见“快速概览”中的 [1] 与 [2]
