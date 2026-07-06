import numpy as np

class PIDController:
    def __init__(self, Kp=None, Ki=None, Kd=None, dt=0.002):
        # Thiết lập các hệ số PID (cần tinh chỉnh cho phù hợp với hệ thống)
        # Các hệ số mặc định này được cấu hình đủ cứng để vượt qua trọng lực
        self.Kp = np.diag([19.0, 19.0]) if Kp is None else Kp
        self.Ki = np.diag([7.0, 3.0]) if Ki is None else Ki
        self.Kd = np.diag([0.8, 0.5]) if Kd is None else Kd
        
        self.dt = dt
        self.integral_error = np.zeros(2)
        self.integral_max = 5.0  # Có thể ghi đè từ bên ngoài (GUI config)

    def compute_control(self, q, dq, q_d, dq_d, ddq_d=None):
        e = q_d - q
        de = dq_d - dq
        
        # Tích phân sai số
        self.integral_error += e * self.dt
        
        # Chống Windup (giới hạn phần tích phân - có thể cấu hình qua GUI)
        self.integral_error = np.clip(self.integral_error, -self.integral_max, self.integral_max)
        
        # Luật điều khiển PID
        tau = self.Kp @ e + self.Ki @ self.integral_error + self.Kd @ de
        return tau
