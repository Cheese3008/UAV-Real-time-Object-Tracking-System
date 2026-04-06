# UAV Real-time Object Tracking System

Hệ thống theo dõi đối tượng thời gian thực cho UAV dựa trên **ROS2**, tập trung vào bài toán **phát hiện đối tượng, theo dõi mục tiêu và ước lượng vị trí mục tiêu từ camera**.

## Tổng quan

Pipeline chính của hệ thống:

**Camera → YOLOv8 Detection → ByteTrack Tracking → Target Pose Estimation → ROS2 Topics → Các module UAV phía sau**

### Chức năng chính

- Phát hiện đối tượng thời gian thực bằng YOLOv8
- Theo dõi mục tiêu qua nhiều khung hình bằng ByteTrack
- Hỗ trợ chọn mục tiêu bằng thao tác click hoặc ROI
- Ước lượng vị trí mục tiêu và publish qua ROS2
- Hỗ trợ ước lượng khoảng cách từ kích thước bounding box
- Publish ảnh debug để quan sát kết quả
- Hỗ trợ tích hợp với các module khác trong hệ thống UAV

---

## Cấu trúc thư mục
```bash
UAV-Real-time-Object-Tracking-System/
├── README.md
└── follow/
    ├── object_lock_tracker_node.py
    ├── models/
    ├── src/
    │   ├── ai_follow/
    │   ├── gimbal_controller/
    │   ├── px4-ros2-interface-lib/
    │   ├── px4_msgs/
    │   ├── rtsp_camera/
    │   ├── target_pose_fusion/
    │   ├── utils/
    │   └── vision_opencv/
    ├── camear_calibration/
    ├── deploy/
    ├── services/
    ├── web/
    │   ├── backend/
    │   └── frontend/
    ├── web_env/
    ├── Makefile
    ├── install_opencv.sh
    ├── install_opencv_4_10_pi5.sh
    └── setup_web.sh
```

---

## Các module chính

### 1. Object Lock Tracker Node

File chính của khối xử lý ảnh là `object_lock_tracker_node.py`.

#### Chức năng

- Subscribe ảnh camera và thông tin camera
- Chạy YOLOv8 để phát hiện đối tượng
- Sử dụng ByteTrack để theo dõi mục tiêu
- Hỗ trợ chọn mục tiêu bằng click hoặc vùng chọn ROI
- Publish pose của mục tiêu và ảnh debug
- Gửi tín hiệu reset và trạng thái mục tiêu cho các node khác

#### Topic đầu vào

- `/camera/image_raw`
- `/camera/camera_info`
- `/click_point`
- `/select_bbox`

#### Topic đầu ra

- `/detecd_pose`
- `/image_proc`
- `/reset`
- `/tag_state`

#### Một số tham số cấu hình chính

- `model_path`
- `conf_thres`
- `z_mode`
- `target_real_height_m`
- `target_class_id`
- `enable_gui`
- `publish_debug_image`

### 2. Target Pose Fusion

Package `target_pose_fusion` dùng để hợp nhất và làm mượt thông tin vị trí mục tiêu, giúp giảm nhiễu từ kết quả thị giác.

Thành phần này phù hợp khi cần đưa dữ liệu mục tiêu từ camera sang các khối điều khiển phía sau một cách ổn định hơn.

### 3. AI Follow Package

Package `ai_follow` chứa cấu trúc ROS2 package cho hệ thống perception bám mục tiêu, bao gồm cấu hình, metadata và phần tích hợp trong workspace ROS2.

### 4. Web Monitoring

Thư mục `web/` bao gồm:

- `backend/` với các file như `app.py`, `ros_bridge.py`
- `frontend/` với giao diện `index.html`

Thành phần này phục vụ cho việc theo dõi hoặc hiển thị hình ảnh xử lý qua giao diện web.

### 5. Deploy và tiện ích

Dự án cũng bao gồm:

- script triển khai trong `deploy/`
- cấu hình service trong `services/`
- script cài OpenCV cho Linux và Raspberry Pi 5
- `Makefile` để build workspace ROS2

---

## Công nghệ sử dụng

- **Python**
- **ROS2**
- **YOLOv8**
- **ByteTrack**
- **OpenCV**
- **Ultralytics**
- **Supervision**
- **C++**
- **PX4 ROS2 Interface**
- **Kalman Filter**

---

## Nguyên lý hoạt động

1. Camera publish ảnh lên ROS2.
2. Node `object_lock_tracker_node.py` nhận ảnh từ camera.
3. YOLOv8 thực hiện phát hiện đối tượng.
4. ByteTrack duy trì ID và theo dõi mục tiêu qua các frame.
5. Hệ thống chọn mục tiêu cần bám và ước lượng vị trí của mục tiêu.
6. Node publish:
   - pose mục tiêu
   - ảnh debug
   - trạng thái tracking
   - tín hiệu reset
7. Các module khác như fusion, gimbal hoặc flight controller có thể subscribe các topic này để sử dụng tiếp.


## Giao tiếp ROS2

### Topic subscribe

- `/camera/image_raw`
- `/camera/camera_info`
- `/click_point`
- `/select_bbox`

### Topic publish

- `/detecd_pose`
- `/image_proc`
- `/reset`
- `/tag_state`


---

## Hướng phát triển

- Tăng độ ổn định khi track lại mục tiêu sau khi bị mất
- Tối ưu tốc độ inference trên thiết bị nhúng

---
