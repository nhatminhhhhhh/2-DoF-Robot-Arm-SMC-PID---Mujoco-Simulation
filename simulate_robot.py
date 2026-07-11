import mujoco
import mujoco.viewer
import numpy as np
import time
import json
import os
from smc_controller import SMCController

# ────────────────────────────────────────────────────────
# --- 0. ĐỌC CẤU HÌNH TỪ GUI (sim_config.json) ---
# ────────────────────────────────────────────────────────
_cfg_path = os.path.join(os.path.dirname(__file__), "sim_config.json")
if os.path.exists(_cfg_path):
    with open(_cfg_path, "r") as _f:
        CFG = json.load(_f)
    print("✅ Đã tải cấu hình từ sim_config.json")
else:
    # Giá trị mặc định nếu chưa chạy GUI
    CFG = {
        "mode": 2, "add_disturbance": True, "duration": 8.0,
        "q_target_1": 1.5, "q_target_2": 1.0, "trajectory_tf": 4.0,
        "sin_amp_1": 0.2, "sin_freq_1": 2.0, "sin_amp_2": 0.1, "sin_freq_2": 4.0,
        "pulse_start": 3.0, "pulse_end": 3.2, "pulse_amp_1": 0.2, "pulse_amp_2": 0.5,
        "smc_lambda_1": 10.0, "smc_lambda_2": 15.0,
        "smc_k_1": 0.1, "smc_k_2": 0.1,
        "smc_eta_1": 50.0, "smc_eta_2": 120.0,
        "smc_phi": 0.1, "smc_use_sat": True,
        "pid_kp_1": 19.0, "pid_kp_2": 19.0,
        "pid_ki_1": 7.0, "pid_ki_2": 3.0,
        "pid_kd_1": 0.8, "pid_kd_2": 0.5, "pid_windup": 5.0,
    }
    print("⚠️  Không tìm thấy sim_config.json, dùng giá trị mặc định.")

# --- 1. TẢI MÔ HÌNH ---
model = mujoco.MjModel.from_xml_path('format_file/2_DOF_Arm/2_DOF_Arm.mjcf')
data = mujoco.MjData(model)

# --- 2. KHỞI TẠO BỘ ĐIỀU KHIỂN TỪ CONFIG ---
MODE           = CFG["mode"]
ADD_DISTURBANCE = CFG["add_disturbance"]

if MODE == 1:
    from pid_controller import PIDController
    import numpy as _np
    controller = PIDController(
        Kp=_np.diag([CFG["pid_kp_1"], CFG["pid_kp_2"]]),
        Ki=_np.diag([CFG["pid_ki_1"], CFG["pid_ki_2"]]),
        Kd=_np.diag([CFG["pid_kd_1"], CFG["pid_kd_2"]]),
        dt=model.opt.timestep
    )
    controller.integral_max = CFG["pid_windup"]
    print(f"Đang chạy ở chế độ: PID Control")
elif MODE == 2:
    controller = SMCController(use_sat=CFG["smc_use_sat"])
    controller.Lambda = np.diag([CFG["smc_lambda_1"], CFG["smc_lambda_2"]])
    controller.K      = np.diag([CFG["smc_k_1"],      CFG["smc_k_2"]])
    controller.eta    = np.diag([CFG["smc_eta_1"],     CFG["smc_eta_2"]])
    controller.phi    = CFG["smc_phi"]
    print(f"Đang chạy ở chế độ: SMC Control (use_sat={CFG['smc_use_sat']})")
else:
    raise ValueError("Mode không hợp lệ. Vui lòng chạy sim_gui.py để cấu hình.")

# --- 3. THIẾT LẬP QUỸ ĐẠO THAM CHIẾU ---
duration = CFG["duration"]
dt = model.opt.timestep   # Bước thời gian
n_steps = int(duration / dt)

# Mảng lưu dữ liệu để vẽ
time_hist = np.linspace(0, duration, n_steps)
# Vị trí góc (Position)
q1_hist = np.zeros(n_steps)
q2_hist = np.zeros(n_steps)
q1_des_hist = np.zeros(n_steps)
q2_des_hist = np.zeros(n_steps)
# Vận tốc góc (Velocity)
dq1_hist = np.zeros(n_steps)
dq2_hist = np.zeros(n_steps)
dq1_des_hist = np.zeros(n_steps)
dq2_des_hist = np.zeros(n_steps)
# Gia tốc góc (Acceleration)
ddq1_des_hist = np.zeros(n_steps)
ddq2_des_hist = np.zeros(n_steps)
# Momen điều khiển
tau1 = np.zeros(n_steps)
tau2 = np.zeros(n_steps)

def get_desired_trajectory(t, q_start, q_desired, T_f=4.0):
    """Tạo quỹ đạo tham chiếu đa thức bậc 5 (Quintic Polynomial) từ góc hiện tại đến góc mong muốn"""
    if t <= T_f:
        a3 = 10 * (q_desired - q_start) / (T_f**3)
        a4 = -15 * (q_desired - q_start) / (T_f**4)
        a5 = 6 * (q_desired - q_start) / (T_f**5)
        
        q_d = a5 * t**5 + a4 * t**4 + a3 * t**3 + q_start
        dq_d = 5 * a5 * t**4 + 4 * a4 * t**3 + 3 * a3 * t**2
        ddq_d = 20 * a5 * t**3 + 12 * a4 * t**2 + 6 * a3 * t
    else:
        # Giữ nguyên tại vị trí đích
        q_d = q_desired.copy()
        dq_d = np.zeros(2)
        ddq_d = np.zeros(2)
        
    return q_d, dq_d, ddq_d
# --- 4. VÒNG LẶP MÔ PHỎNG ---
# Tải tư thế ban đầu "home" từ keyframe đã định nghĩa trong MJCF
# Keyframe này đảm bảo tất cả các khớp nằm trong giới hạn hợp lệ
home_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
if home_key_id >= 0:
    mujoco.mj_resetDataKeyframe(model, data, home_key_id)
else:
    # Fallback: kẹp thủ công vào giới hạn nếu không tìm thấy keyframe
    for j in range(model.njnt):
        if data.qpos[j] < model.jnt_range[j][0]:
            data.qpos[j] = model.jnt_range[j][0]
        elif data.qpos[j] > model.jnt_range[j][1]:
            data.qpos[j] = model.jnt_range[j][1]

mujoco.mj_forward(model, data)
q_start = data.qpos.copy()

# Góc mục tiêu đọc từ cấu hình GUI
q_target = np.array([CFG["q_target_1"], CFG["q_target_2"]])
_T_f = CFG["trajectory_tf"]

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    for i in range(n_steps):
        t = i * dt
        
        # Lấy trạng thái hiện tại
        q = data.qpos.copy()
        dq = data.qvel.copy()

        # Lấy quỹ đạo tham chiếu từ góc xuất phát (dùng T_f từ config)
        q_d, dq_d, ddq_d = get_desired_trajectory(t, q_start, q_target, T_f=_T_f)

        # Tính mô-men điều khiển
        tau = controller.compute_control(q, dq, q_d, dq_d, ddq_d)

        # Giả lập nhiễu mô-men ngoại lực (External Torque Disturbance)
        if ADD_DISTURBANCE:
            tau_dist = np.zeros(2)
            # Nhiễu hình sin (đọc từ config)
            tau_dist[0] = CFG["sin_amp_1"] * np.sin(CFG["sin_freq_1"] * t)
            tau_dist[1] = CFG["sin_amp_2"] * np.sin(CFG["sin_freq_2"] * t)
            
            # Xung nhiễu đột ngột (đọc từ config)
            if CFG["pulse_start"] < t < CFG["pulse_end"]:
                tau_dist[0] += CFG["pulse_amp_1"]
                tau_dist[1] += CFG["pulse_amp_2"]
                
            # Tổng hợp tín hiệu điều khiển và nhiễu đưa vào robot
            data.ctrl[:] = tau + tau_dist
        else:
            data.ctrl[:] = tau

        # Lưu dữ liệu
        q1_hist[i] = q[0]
        q2_hist[i] = q[1]
        q1_des_hist[i] = q_d[0]
        q2_des_hist[i] = q_d[1]
        dq1_hist[i] = dq[0]
        dq2_hist[i] = dq[1]
        dq1_des_hist[i] = dq_d[0]
        dq2_des_hist[i] = dq_d[1]
        ddq1_des_hist[i] = ddq_d[0]
        ddq2_des_hist[i] = ddq_d[1]
        tau1[i] = tau[0]
        tau2[i] = tau[1]

        # Tiến một bước mô phỏng
        mujoco.mj_step(model, data)

        # Cập nhật hiển thị
        viewer.sync()
        time.sleep(dt)  # Đồng bộ thời gian thực

# --- 5. VẼ BIỂU ĐỒ KẾT QUẢ ---
import matplotlib.pyplot as plt

plt.figure(figsize=(14, 8))

label_q1_act = 'q1 thực tế'
label_q2_act = 'q2 thực tế'

dist_str = "Noise: Active" if ADD_DISTURBANCE else "Noise: None"

if MODE == 2:
    sat_str = "sat" if CFG['smc_use_sat'] else "sign"
    label_q1_act += f" (SMC: Lambda={CFG['smc_lambda_1']}, K={CFG['smc_k_1']}, Eta={CFG['smc_eta_1']}, Phi={CFG['smc_phi']}, func={sat_str} | {dist_str})"
    label_q2_act += f" (SMC: Lambda={CFG['smc_lambda_2']}, K={CFG['smc_k_2']}, Eta={CFG['smc_eta_2']}, Phi={CFG['smc_phi']}, func={sat_str} | {dist_str})"
else:
    label_q1_act += f" ({dist_str})"
    label_q2_act += f" ({dist_str})"

label_q1_ref = f"q1 tham chiếu (Target: {CFG['q_target_1']} rad)"
label_q2_ref = f"q2 tham chiếu (Target: {CFG['q_target_2']} rad)"

plt.subplot(2, 1, 1)
plt.plot(time_hist, q1_hist, label=label_q1_act, linewidth=2)
plt.plot(time_hist, q1_des_hist, '--', label=label_q1_ref, linewidth=2)
plt.legend(fontsize=10)
plt.grid()
plt.ylabel('Góc khớp 1 (rad)', fontsize=12)
plt.title('Quỹ đạo góc khớp (Reference vs Actual)', fontsize=14)

plt.subplot(2, 1, 2)
plt.plot(time_hist, q2_hist, label=label_q2_act, linewidth=2)
plt.plot(time_hist, q2_des_hist, '--', label=label_q2_ref, linewidth=2)
plt.legend(fontsize=10)
plt.grid()
plt.xlabel('Thời gian (s)', fontsize=12)
plt.ylabel('Góc khớp 2 (rad)', fontsize=12)
plt.tight_layout()
plt.show()

# Biểu đồ sai số
plt.figure(figsize=(14, 8))
plt.subplot(2, 1, 1)
plt.plot(time_hist, q1_des_hist - q1_hist, label='q1 sai số', color='red', linewidth=2)
plt.legend(fontsize=12)
plt.grid()
plt.ylabel('Sai số khớp 1 (rad)', fontsize=12)
plt.title('Sai số bám quỹ đạo (Tracking Error)', fontsize=14)

plt.subplot(2, 1, 2)
plt.plot(time_hist, q2_des_hist - q2_hist, label='q2 sai số', color='red', linewidth=2)
plt.legend(fontsize=12)
plt.grid()
plt.xlabel('Thời gian (s)', fontsize=12)
plt.ylabel('Sai số khớp 2 (rad)', fontsize=12)
plt.tight_layout()
plt.show()

# --- Biểu đồ vận tốc góc (Angular Velocity) ---
plt.figure(figsize=(14, 8))
plt.subplot(2, 1, 1)
plt.plot(time_hist, dq1_hist,     label='dq1 thực tế',    linewidth=2)
plt.plot(time_hist, dq1_des_hist, '--', label='dq1 tham chiếu', linewidth=2)
plt.legend(fontsize=12)
plt.grid()
plt.ylabel('Vận tốc góc khớp 1 (rad/s)', fontsize=12)
plt.title('Vận tốc góc (Angular Velocity – Reference vs Actual)', fontsize=14)

plt.subplot(2, 1, 2)
plt.plot(time_hist, dq2_hist,     label='dq2 thực tế',    linewidth=2)
plt.plot(time_hist, dq2_des_hist, '--', label='dq2 tham chiếu', linewidth=2)
plt.legend(fontsize=12)
plt.grid()
plt.xlabel('Thời gian (s)', fontsize=12)
plt.ylabel('Vận tốc góc khớp 2 (rad/s)', fontsize=12)
plt.tight_layout()
plt.show()

# --- Biểu đồ gia tốc góc – chỉ tham chiếu (từ trajectory) ---
# Gia tốc thực tế tính bằng đạo hàm số từ vận tốc đo được
ddq1_act = np.gradient(dq1_hist, time_hist)
ddq2_act = np.gradient(dq2_hist, time_hist)

plt.figure(figsize=(14, 8))
plt.subplot(2, 1, 1)
plt.plot(time_hist, ddq1_act,     label='ddq1 thực tế (d/dt dq)',  linewidth=2)
plt.plot(time_hist, ddq1_des_hist, '--', label='ddq1 tham chiếu (trajectory)', linewidth=2)
plt.legend(fontsize=12)
plt.grid()
plt.ylabel('Gia tốc góc khớp 1 (rad/s²)', fontsize=12)
plt.title('Gia tốc góc (Angular Acceleration – Reference vs Actual)', fontsize=14)
margin1 = max(np.max(np.abs(ddq1_des_hist)) * 0.2, 0.1)
plt.ylim(np.min(ddq1_des_hist) - margin1, np.max(ddq1_des_hist) + margin1)

plt.subplot(2, 1, 2)
plt.plot(time_hist, ddq2_act,     label='ddq2 thực tế (d/dt dq)',  linewidth=2)
plt.plot(time_hist, ddq2_des_hist, '--', label='ddq2 tham chiếu (trajectory)', linewidth=2)
plt.legend(fontsize=12)
plt.grid()
plt.xlabel('Thời gian (s)', fontsize=12)
plt.ylabel('Gia tốc góc khớp 2 (rad/s²)', fontsize=12)
margin2 = max(np.max(np.abs(ddq2_des_hist)) * 0.2, 0.1)
plt.ylim(np.min(ddq2_des_hist) - margin2, np.max(ddq2_des_hist) + margin2)
plt.tight_layout()
plt.show()

# --- Biểu đồ momen điều khiển ---
plt.figure(figsize=(14, 8))
plt.subplot(2, 1, 1)
plt.plot(time_hist, tau1, label='tau1', linewidth=2)
plt.legend(fontsize=12)
plt.grid()
plt.ylabel('Momen điều khiển khớp 1 (Nm)', fontsize=12)
plt.title('Momen điều khiển (Control Input)', fontsize=14)

plt.subplot(2, 1, 2)
plt.plot(time_hist, tau2, label='tau2', linewidth=2)
plt.legend(fontsize=12)
plt.grid()
plt.xlabel('Thời gian (s)', fontsize=12)
plt.ylabel('Momen điều khiển khớp 2 (Nm)', fontsize=12)
plt.tight_layout()
plt.show()