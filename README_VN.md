# SMC Robot Arm Simulation 🤖

Mô phỏng hệ thống điều khiển cánh tay robot 2 bậc tự do (2-DOF) sử dụng **Bộ điều khiển Trượt (Sliding Mode Control - SMC)** và **PID** trong môi trường vật lý **MuJoCo**. Đi kèm với giao diện bảng điều khiển GUI để tùy chỉnh các thông số một cách trực quan.

## Tính năng nổi bật
- So sánh hiệu năng giữa PID và SMC khi robot bị tác động bởi nhiễu (nhiễu hình sin, xung nhiễu va chạm).
- Mô phỏng động lực học chính xác của robot 2-DOF với số liệu ma trận quán tính được trích xuất trực tiếp từ bản thiết kế CAD.
- Bảng điều khiển (GUI) trực quan: dễ dàng tinh chỉnh chế độ, cấu hình quỹ đạo đích, thông số nhiễu ngoại lực, và siêu tham số của bộ điều khiển.
- Biểu đồ phân tích trực quan: Quỹ đạo thực tế vs tham chiếu, Sai số bám (Tracking Error), và Momen điều khiển (Control Input).

---

## Hướng dẫn Cài đặt & Thiết lập

### 1. Yêu cầu hệ thống
- Hệ điều hành: Windows, macOS, hoặc Linux.
- Cài đặt sẵn **Python 3.9 trở lên** (Khuyến khích dùng Python 3.10 hoặc 3.11).
  > **Lưu ý cho người dùng mới:** Nếu bạn chưa có Python, hãy tải tại [python.org/downloads](https://www.python.org/downloads/). Khi chạy file cài đặt, hãy nhớ **TÍCH VÀO Ô "Add python.exe to PATH"** trước khi nhấn Install.

### 2. Tải mã nguồn (Clone Project)
Mở Terminal (hoặc Powershell/Command Prompt) và chạy lệnh sau:
```bash
git clone https://github.com/nhatminhhhhhh/2-DoF-Robot-Arm-SMC-PID---Mujoco-Simulation.git
cd SMC_Robot_Arm
```

### 3. Tạo môi trường ảo (Virtual Environment)
Môi trường ảo giúp tách biệt các thư viện của project này với các project khác trên máy bạn.
```bash
# Tạo môi trường ảo có tên là mujoco_env
python -m venv mujoco_env

# Kích hoạt môi trường (Dành cho Windows)
.\mujoco_env\Scripts\activate

# Kích hoạt môi trường (Dành cho macOS/Linux)
source mujoco_env/bin/activate
```
*(Nếu kích hoạt thành công, bạn sẽ thấy chữ `(mujoco_env)` xuất hiện ở đầu dòng lệnh).*

### 4. Cài đặt các thư viện cần thiết
Đảm bảo bạn đang ở trong môi trường ảo, sau đó chạy lệnh cài đặt toàn bộ gói thư viện:
```bash
pip install -r requirements.txt
```

---

## Hướng dẫn Sử dụng

Để khởi động Bảng điều khiển trung tâm (Config Panel), hãy chạy lệnh:
```bash
python sim_gui.py
```

1. **Thiết lập:** Thay đổi các thông số điều khiển (PID/SMC), thêm nhiễu hoặc cấu hình quỹ đạo trực tiếp trên giao diện.
2. **Lưu:** Nhấn nút **💾 SAVE** để lưu file cấu hình `sim_config.json`.
3. **Khởi chạy:** Nhấn nút **🚀 RUN** để tiến hành mô phỏng. Cửa sổ 3D của MuJoCo sẽ mở ra, sau khi kết thúc các đồ thị báo cáo kết quả sẽ tự động hiện lên.

---
*Designed by: Nhat_Minh*
