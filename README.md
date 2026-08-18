# esp32s3_balance_scooter_yolo11_detect_fire

<p style="font-size:20px">
Based on ESP32-S3 & R5 Pro self-balancing vehicle, dual-loop PID balance and independent steering loop, wireless control via WeChat BLE mini-program.(基于ESP32-S3与R5 Pro自平衡小车，双环PID平衡+独立转向环，微信BLE小程序无线控制。); Real-time fire & smoke detection with YOLO11-Nano, dual-thread HTTP camera frame capture, anti-shake alert and visualization window(YOLO11-Nano 远程摄像头火灾烟雾实时检测程序，双线程分离采集与推理，带防抖告警可视化窗口)
</p>

<p style="font-size:12px">
balabce_scooterv4.1: This project serves as a preliminary implementation of a two-wheeled self-balancing vehicle, where core control functions are basically realized, and multiple aspects of the scheme remain to be optimized.
  (本项目为两轮自平衡小车基础实现版本，完成核心控制功能的初步开发，整体方案仍存在优化空间。)
</p>

<p style="font-size:12px">
balance_scooter_v4.3: Upgraded firmware for ESP32-S3 two-wheeled self-balancing robot. The project adopts FreeRTOS multi-task dual-core scheduling. Real-time balance control runs independently on Core1, and human-computer interaction logic runs on Core0, which effectively eliminates the timing jitter caused by screen refresh and Bluetooth communication in the single-core version. Supports wireless control via BLE mini-program, cascaded angle-speed differential steering PID algorithm.
(balance_scooter_v4.3：ESP32-S3两轮自平衡机器人升级固件。项目采用FreeRTOS多任务双核调度方案，实时平衡控制独立运行于Core1，人机交互逻辑运行在Core0，有效消除单核版本中屏幕刷新、蓝牙通信带来的控制时序抖动；支持微信BLE小程序无线操控，搭载串级角度-速度差速转向PID算法。)
</p>

<p style="font-size:20px">
esp32s3_balance_scooter_v6.1: Abandon external HC-04 serial Bluetooth module, fully switch to ESP32-S3 native NimBLE protocol stack, optimize FreeRTOS dual-core multi-task scheduling, fix intermittent BLE command loss bug, integrate YOLO11-Nano dual-thread real-time fire and smoke detection module.
(v6.1版本：弃用外置HC-04串口蓝牙模块，全面切换ESP32-S3原生NimBLE协议栈，优化FreeRTOS双核多任务调度，修复蓝牙指令偶发丢失缺陷，集成YOLO11-Nano双线程火情烟雾实时检测模块。)
</p>

<p style="font-size:12px">
1. Bluetooth architecture reconstruction: Completely remove dependent HC-04 serial Bluetooth hardware, adopt on-chip NimBLE BLE, add FreeRTOS FIFO command queue to cache Bluetooth data frames, solve the problem of command no response caused by single global buffer coverage under rapid instruction transmission; lock BLE connection interval via PPCP parameters to eliminate remote control lag caused by Android power-saving strategy.
（蓝牙架构重构：彻底移除依赖的HC-04串口蓝牙硬件，采用片内NimBLE蓝牙，新增FreeRTOS环形指令队列缓存蓝牙数据包，解决高速下发指令时单全局缓冲区覆盖导致指令无响应问题；通过PPC参数锁定蓝牙连接间隔，消除安卓省电策略带来遥控延迟。）
</p>

<p style="font-size:12px">
2. Dual-core scheduling optimization: Rearrange task core binding rules, balance PID control, BLE parsing and OLED display tasks run on Core1 uniformly, Core0 only undertakes NimBLE underlying RF controller and low-frequency hardware interrupts, eliminate WDT infinite reboot crash caused by I2C OLED interrupt congestion; add ANGLE_LIMIT macro to standardize tilt rollover protection threshold, retain ENABLE_OLED_DISPLAY conditional compilation macro for one-click shielding of blocking I2C screen logic.
（双核调度优化：重新规划任务内核绑定规则，平衡PID控制、蓝牙解析、OLED显示任务统一运行于Core1，Core0仅承载NimBLE底层射频控制器与低频硬件中断，解决I2C OLED中断堆积引发看门狗循环重启故障；新增ANGLE_LIMIT宏标准化倾倒保护阈值，保留ENABLE_OLED_DISPLAY条件编译宏，可一键屏蔽阻塞I2C屏幕逻辑。）
</p>


<p style="font-size:12px">
my_wifi_esp32cam_v3.2: A lightweight fire and smoke recognition program powered by YOLO11-Nano, designed for ESP32-S3 camera hardware. It adopts a dual-thread decoupling architecture: an independent background thread continuously fetches frames from the HTTP video stream output by ESP32-S3, while the main thread executes model inference and visualization to prevent network IO blocking detection logic. The pre-trained YOLO11-Nano fire and smoke detection weights are sourced from the open-source repository: https://github.com/sayedgamal99/Real-Time-Smoke-Fire-Detection-YOLO11.
  (my_wifi_esp32cam_v3.2：基于 YOLO11-Nano 轻量化模型、适配 ESP32-S3 摄像头硬件实现远程火情烟雾实时检测。程序采用双线程解耦架构：独立后台线程持续拉取 ESP32-S3 输出的 HTTP 视频流图像，主线程执行模型推理与画面渲染，避免网络 IO 阻塞检测流程。项目所使用的 YOLO11-Nano 火焰烟雾预训练权重来源于开源仓库：https://github.com/sayedgamal99/Real-Time-Smoke-Fire-Detection-YOLO11。)
</p>
