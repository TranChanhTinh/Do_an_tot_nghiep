# Mô hình xe ADAS – README

Dự án xây dựng mô hình xe tích hợp hệ thống hỗ trợ lái xe nâng cao (ADAS) với mục tiêu tái hiện các chức năng thông minh trong môi trường phòng thí nghiệm. Hệ thống tập trung vào nhận diện làn đường, biển báo, kiểm soát tốc độ và phanh tránh va chạm.

## 1. Kiến trúc hệ thống

* **Raspberry Pi 5**: Xử lý ảnh, chạy mô hình YOLOv11n và các thuật toán điều khiển.
* **ESP32**: Điều khiển động cơ, servo qua mạch cầu H.
* **Cảm biến**: HC-SR04 đo khoảng cách.
* **Camera**: Cung cấp dữ liệu hình ảnh thời gian thực cho Pi.

## 2. Chức năng chính

* Giữ làn và căn chỉnh hướng lái.
* Nhận diện biển báo giao thông.
* Kiểm soát tốc độ dựa trên phản hồi cảm biến.
* Phanh tự động khi phát hiện vật cản.

## 3. Hiệu năng

Hệ thống hoạt động ổn định trong các thử nghiệm cơ bản, xử lý tốt các tình huống đơn giản.

## 4. Mục tiêu học thuật

Dự án là nền tảng để tiếp cận các công nghệ xe tự hành và hệ thống hỗ trợ người lái, phù hợp nghiên cứu và đào tạo trong lĩnh vực nhúng, xử lý ảnh và robot.
