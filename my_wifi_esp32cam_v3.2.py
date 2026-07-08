from ultralytics import YOLO
import cv2
import requests
import numpy as np
import time
import threading

CAP_URL = "http://192.168.5.1/capture"
model_path = r"D:\DeepLeraning_AI_code\Yolo\Yolo11\ultralytics-8.3.163\Real-Time-Smoke-Fire-Detection-YOLO11-main\models\best_nano_111.pt"
model = YOLO(model_path)

# 全局共享帧 + 线程锁，保证画面同步
latest_frame = None
frame_lock = threading.Lock()
session = requests.Session()
session.headers["Connection"] = "close"

# 告警防抖标记，避免重复刷屏
last_fire_alert = 0
last_smoke_alert = 0
alert_interval = 2.0  # 2秒内只打印一次告警

# 后台持续采集线程（独立拉取图像，不占用推理线程）
def capture_thread():
    global latest_frame
    while True:
        try:
            resp = session.get(CAP_URL, timeout=6)
            if len(resp.content) < 1000:
                time.sleep(0.05)
                continue
            img_buf = np.frombuffer(resp.content, np.uint8)
            frame = cv2.imdecode(img_buf, cv2.IMREAD_COLOR)
            if frame is not None:
                with frame_lock:
                    latest_frame = frame
        except Exception:
            pass
        time.sleep(0.02)  # 极短间隔持续抢最新帧

if __name__ == "__main__":
    print("🔥 Fire & Smoke Detection Running Smooth Mode")
    win_title = "Fire Smoke Detect Window"
    cv2.namedWindow(win_title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_title, 960, 540)
    TARGET_W, TARGET_H = 960, 540

    # 启动采集后台线程
    t = threading.Thread(target=capture_thread, daemon=True)
    t.start()
    time.sleep(1)  # 等待线程初始化

    while True:
        current_frame = None
        # 取出最新帧，无帧则跳过本次循环
        with frame_lock:
            if latest_frame is not None:
                current_frame = latest_frame.copy()
        if current_frame is None:
            cv2.waitKey(1)
            continue

        # 轻量化推理，关闭日志输出提速
        res = model(current_frame, conf=0.35, iou=0.1, imgsz=480, verbose=False)
        show = res[0].plot()
        show = cv2.resize(show, (TARGET_W, TARGET_H), interpolation=cv2.INTER_LINEAR)
        cv2.imshow(win_title, show)

        # 防抖告警，防止大量打印造成卡顿
        boxes = res[0].boxes
        now = time.time()
        if len(boxes) > 0:
            cls_ids = boxes.cls.cpu().numpy()
            has_fire = 0 in cls_ids
            has_smoke = 1 in cls_ids

            if has_fire and now - last_fire_alert > alert_interval:
                print("⚠️ ALERT: FIRE DETECTED!")
                last_fire_alert = now
            if has_smoke and now - last_smoke_alert > alert_interval:
                print("⚠️ ALERT: SMOKE DETECTED!")
                last_smoke_alert = now

        # 仅窗口刷新，无强制延时
        key = cv2.waitKey(1)
        if key & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()