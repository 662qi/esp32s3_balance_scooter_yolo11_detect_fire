# esp32s3_balance_scooter_yolo11_detect_fire

<p style="font-size:20px">
Based on ESP32-S3 & R5 Pro self-balancing vehicle, dual-loop PID balance and independent steering loop, wireless control via WeChat BLE mini-program.(基于ESP32-S3与R5 Pro自平衡小车，双环PID平衡+独立转向环，微信BLE小程序无线控制。); Real-time fire & smoke detection with YOLO11-Nano, dual-thread HTTP camera frame capture, anti-shake alert and visualization window(YOLO11-Nano 远程摄像头火灾烟雾实时检测程序，双线程分离采集与推理，带防抖告警可视化窗口)
</p>

<p style="font-size:12px">
balabce_scooterv4.1: This project serves as a preliminary implementation of a two-wheeled self-balancing vehicle, where core control functions are basically realized, and multiple aspects of the scheme remain to be optimized.(本项目为两轮自平衡小车基础实现版本，完成核心控制功能的初步开发，整体方案仍存在优化空间。)
</p>

<p style="font-size:12px">
my_wifi_esp32cam_v3.2: A lightweight fire and smoke recognition program powered by YOLO11-Nano, adopting a dual-thread decoupling architecture: an independent background thread continuously fetches frames from remote HTTP camera, while the main thread executes model inference and visualization to prevent network IO blocking detection logic.(基于 YOLO11-Nano 轻量化模型实现远程摄像头火情实时检测，采用双线程分离架构，独立线程持续拉取 HTTP 摄像头图像，主线程负责目标推理与画面渲染，避免网络 IO 阻塞推理流程。)
</p>
