import numpy as np

class SMCController:
    def __init__(self, use_sat=True):
        self.use_sat = use_sat               # Tùy chọn: True = dùng hàm sat, False = dùng hàm sign
        # Tham số điều khiển (cần tinh chỉnh)
        self.Lambda = np.diag([10, 15])    # Độ rộng mặt trượt
        self.K = np.diag([0.1, 0.1])         # Độ lợi tỷ lệ 
        self.eta = np.diag([50.0, 120.0])       # Độ lợi chống nhiễu 
        self.phi = 0.1                      # Bề dày lớp biên (giảm chattering)

        # Tham số động lực học robot (khớp với model CAD/ .mjcf)
        self.m1, self.m2 = 1.5668, 0.9700        # Khối lượng khâu 1, 2 (kg)
        self.l1 = 0.12                           # Chiều dài khâu 1 (m)
        self.lc1, self.lc2 = 0.0538, 0.0718      # Khoảng cách từ trục quay tới CoM (m)
        
        # Ma trận Moment quán tính 3x3 tại Center of Mass (kg.m^2) - Lấy số liệu chính xác từ URDF (chưa bị làm tròn như UI Fusion)
        self.I1_matrix = np.array([
            [0.000539, 0.0, 0.0],
            [0.0, 0.002019, 0.0],
            [0.0, 0.0, 0.001856]
        ])
        self.I2_matrix = np.array([
            [0.000145, 0.0, 0.0],
            [0.0, 0.002187, 0.0],
            [0.0, 0.0, 0.002187]
        ])
        # Vì robot phẳng (chỉ xoay quanh trục Z), động lực học chỉ bị ảnh hưởng bởi thành phần Izz (hàng 3, cột 3)
        self.I1 = self.I1_matrix[2, 2]
        self.I2 = self.I2_matrix[2, 2]
        
        self.g = 9.81

    def _robot_dynamics(self, q, dq):
        """Tính ma trận M(q), C(q, dq), G(q)"""
        q1, q2 = q[0], q[1]
        dq1, dq2 = dq[0], dq[1]

        # Ma trận khối lượng M(q)
        M11 = self.m1*self.lc1**2 + self.m2*(self.l1**2 + self.lc2**2 + 2*self.l1*self.lc2*np.cos(q2)) + self.I1 + self.I2
        M12 = self.m2*(self.lc2**2 + self.l1*self.lc2*np.cos(q2)) + self.I2
        M21 = M12
        M22 = self.m2*self.lc2**2 + self.I2
        M = np.array([[M11, M12], [M21, M22]])

        # Ma trận Coriolis và lực hướng tâm C(q, dq)
        h = -self.m2 * self.l1 * self.lc2 * np.sin(q2)
        C = np.array([[h*dq2, h*(dq1+dq2)], [-h*dq1, 0]])

        # Vector trọng lực G(q)
        G1 = (self.m1*self.lc1 + self.m2*self.l1)*self.g*np.cos(q1) + self.m2*self.lc2*self.g*np.cos(q1+q2)
        G2 = self.m2*self.lc2*self.g*np.cos(q1+q2)
        G = np.array([G1, G2])

        return M, C, G

    def compute_control(self, q, dq, q_d, dq_d, ddq_d):
        """Tính mô-men điều khiển tau"""
        # Sai số
        e = q - q_d
        de = dq - dq_d

        # Mặt trượt
        s = de + self.Lambda @ e

        # Động lực học robot
        M, C, G = self._robot_dynamics(q, dq)

        # Chức năng chọn hàm chống chattering:
        if self.use_sat:
            switching_term = np.clip(s / self.phi, -1, 1)  # Hàm sat giảm chattering
        else:
            switching_term = np.sign(s)                    # Hàm sign cơ bản

        # Luật điều khiển SMC
        tau = M @ (ddq_d - self.Lambda @ de - self.K @ s - self.eta @ switching_term) + C @ dq + G
        return tau