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

<p style="font-size:12px">
my_wifi_esp32cam_v3.2: A lightweight fire and smoke recognition program powered by YOLO11-Nano, designed for ESP32-S3 camera hardware. It adopts a dual-thread decoupling architecture: an independent background thread continuously fetches frames from the HTTP video stream output by ESP32-S3, while the main thread executes model inference and visualization to prevent network IO blocking detection logic. The pre-trained YOLO11-Nano fire and smoke detection weights are sourced from the open-source repository: https://github.com/sayedgamal99/Real-Time-Smoke-Fire-Detection-YOLO11.
  (my_wifi_esp32cam_v3.2：基于 YOLO11-Nano 轻量化模型、适配 ESP32-S3 摄像头硬件实现远程火情烟雾实时检测。程序采用双线程解耦架构：独立后台线程持续拉取 ESP32-S3 输出的 HTTP 视频流图像，主线程执行模型推理与画面渲染，避免网络 IO 阻塞检测流程。项目所使用的 YOLO11-Nano 火焰烟雾预训练权重来源于开源仓库：https://github.com/sayedgamal99/Real-Time-Smoke-Fire-Detection-YOLO11。)
</p>
