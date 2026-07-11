import tkinter as tk
from tkinter import ttk, messagebox
import json
import os

# --- Màu sắc & Font cho theme không gian / tech ---
BG_DARK    = "#0B0C10" # Đen không gian sâu
BG_PANEL   = "#1F2833" # Xanh xám kim loại
BG_CARD    = "#111A22" # Đen xanh mờ (kính buồng lái)
ACCENT     = "#66FCF1" # Xanh Neon (Cyan)
ACCENT2    = "#45A29E" # Xanh lam nhạt
TEXT_WHITE = "#C5C6C7" # Trắng xám kim loại
TEXT_GRAY  = "#8B959A" # Xám nhạt
GREEN      = "#39FF14" # Xanh lá Neon
YELLOW     = "#FCE205" # Vàng Neon

FONT_TITLE  = ("Consolas", 15, "bold")
FONT_HEADER = ("Consolas", 12, "bold")
FONT_LABEL  = ("Consolas", 11)
FONT_SMALL  = ("Consolas", 10)

CONFIG_FILE = os.path.join(os.path.dirname(__file__), "sim_config.json")

DEFAULT_CONFIG = {
    "mode": 2,
    "add_disturbance": True,
    "duration": 8.0,
    "q_target_1": 1.5,
    "q_target_2": 1.0,
    "trajectory_tf": 4.0,
    "sin_amp_1": 0.2,
    "sin_freq_1": 2.0,
    "sin_amp_2": 0.1,
    "sin_freq_2": 4.0,
    "pulse_start": 3.0,
    "pulse_end": 3.2,
    "pulse_amp_1": 0.2,
    "pulse_amp_2": 0.5,
    "smc_lambda_1": 10.0,
    "smc_lambda_2": 15.0,
    "smc_k_1": 0.1,
    "smc_k_2": 0.1,
    "smc_eta_1": 50.0,
    "smc_eta_2": 100.0,
    "smc_phi": 0.1,
    "smc_use_sat": True,
    "pid_kp_1": 19.0,
    "pid_kp_2": 19.0,
    "pid_ki_1": 7.0,
    "pid_ki_2": 3.0,
    "pid_kd_1": 0.8,
    "pid_kd_2": 0.5,
    "pid_windup": 5.0,
}


def load_config():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            cfg = json.load(f)
        # Merge with defaults to handle missing keys
        merged = DEFAULT_CONFIG.copy()
        merged.update(cfg)
        return merged
    return DEFAULT_CONFIG.copy()


def save_config(cfg):
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=4)


class LabelEntry(tk.Frame):
    """Widget kết hợp Label + Entry với style đẹp"""
    def __init__(self, parent, label, default, unit="", width=10, **kwargs):
        super().__init__(parent, bg=BG_CARD, **kwargs)
        tk.Label(self, text=label, font=FONT_LABEL, bg=BG_CARD, fg=TEXT_WHITE,
                 anchor="w", width=28).pack(side="left", padx=(6, 2))
        self.var = tk.StringVar(value=str(default))
        entry = tk.Entry(self, textvariable=self.var, font=FONT_LABEL,
                         bg=BG_DARK, fg=ACCENT2, insertbackground=ACCENT2,
                         relief="flat", width=width, justify="center",
                         highlightthickness=1, highlightcolor=ACCENT2,
                         highlightbackground=BG_PANEL)
        entry.pack(side="left", padx=4, ipady=3)
        if unit:
            tk.Label(self, text=unit, font=FONT_SMALL, bg=BG_CARD, fg=TEXT_GRAY).pack(side="left")

    def get(self):
        return self.var.get()

    def set(self, val):
        self.var.set(str(val))


class SectionHeader(tk.Label):
    """Tiêu đề section"""
    def __init__(self, parent, text, color=ACCENT, **kwargs):
        super().__init__(parent, text=f" [>] {text}", font=FONT_TITLE,
                         bg=BG_PANEL, fg=color, anchor="w",
                         pady=6, padx=4, **kwargs)


class SimConfigGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Simulation – Configuration Panel")
        self.root.configure(bg=BG_DARK)
        self.root.resizable(True, True)
        self.root.geometry("860x900")
        self.cfg = load_config()
        self._build_ui()
        self._load_values()

    # ──────────────────────────────────────────────
    # UI BUILD
    # ──────────────────────────────────────────────
    def _build_ui(self):
        # ---- Nhat Minh dep trai 123 ----
        title_bar = tk.Frame(self.root, bg=BG_PANEL, height=50)
        title_bar.pack(fill="x")
        tk.Label(title_bar, text="🛸 ORBITAL COMMAND :: SMC ROBOT ARM CONFIG",
                 font=("Consolas", 16, "bold"), bg=BG_PANEL, fg=ACCENT,
                 pady=10).pack(side="left", padx=16)
        tk.Label(title_bar, text="Designed by: Nhat_Minh",
                 font=("Consolas", 11, "italic"), bg=BG_PANEL, fg=TEXT_GRAY,
                 pady=10).pack(side="right", padx=16)

        # ---- Scrollable canvas ----
        canvas_frame = tk.Frame(self.root, bg=BG_DARK)
        canvas_frame.pack(fill="both", expand=True)
        canvas = tk.Canvas(canvas_frame, bg=BG_DARK, highlightthickness=0)
        scrollbar = ttk.Scrollbar(canvas_frame, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        self.scroll_frame = tk.Frame(canvas, bg=BG_DARK)
        canvas_window = canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw")

        def _on_configure(e):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfig(canvas_window, width=canvas.winfo_width())

        self.scroll_frame.bind("<Configure>", _on_configure)
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(canvas_window, width=e.width))
        canvas.bind_all("<MouseWheel>", lambda e: canvas.yview_scroll(-1*(e.delta//120), "units"))

        # ---- Sections ----
        self._build_general_section()
        self._build_trajectory_section()
        self._build_disturbance_section()
        self._build_smc_section()
        self._build_pid_section()

        # ---- Action buttons ----
        btn_frame = tk.Frame(self.root, bg=BG_DARK, pady=10)
        btn_frame.pack(fill="x")
        self._make_button(btn_frame, "💾 SAVE", self._save, ACCENT, BG_DARK).pack(side="left", padx=14)
        self._make_button(btn_frame, "🔄 RESET", self._reset, BG_CARD, TEXT_WHITE).pack(side="left", padx=4)
        self._make_button(btn_frame, "🚀 RUN", self._run_sim, GREEN, BG_DARK).pack(side="right", padx=14)

    def _make_button(self, parent, text, cmd, bg, fg):
        return tk.Button(parent, text=text, command=cmd,
                         font=("Consolas", 12, "bold"),
                         bg=bg, fg=fg, activebackground=TEXT_WHITE,
                         activeforeground=BG_DARK, relief="flat",
                         padx=16, pady=8, cursor="hand2")

    # ──────────────────────────────────────────────
    # SECTION: GENERAL
    # ──────────────────────────────────────────────
    def _build_general_section(self):
        f = self.scroll_frame
        SectionHeader(f, "GENERAL SETTINGS", ACCENT).pack(fill="x", padx=8, pady=(12, 2))
        card = tk.Frame(f, bg=BG_CARD, bd=0, pady=8, padx=8)
        card.pack(fill="x", padx=8, pady=4)

        # Mode selection
        mode_row = tk.Frame(card, bg=BG_CARD)
        mode_row.pack(fill="x", pady=4)
        tk.Label(mode_row, text="Chế độ điều khiển:", font=FONT_LABEL, bg=BG_CARD, fg=TEXT_WHITE, width=24, anchor="w").pack(side="left", padx=6)
        self.mode_var = tk.IntVar(value=2)
        for val, lbl in [(1, "Mode 1 – PID"), (2, "Mode 2 – SMC")]:
            tk.Radiobutton(mode_row, text=lbl, variable=self.mode_var, value=val,
                           font=FONT_LABEL, bg=BG_CARD, fg=TEXT_WHITE,
                           selectcolor=BG_DARK, activebackground=BG_CARD,
                           activeforeground=ACCENT2).pack(side="left", padx=10)

        self.duration_entry = LabelEntry(card, "Thời gian mô phỏng:", self.cfg["duration"], "s")
        self.duration_entry.pack(fill="x", pady=3)

    # ──────────────────────────────────────────────
    # SECTION: TRAJECTORY
    # ──────────────────────────────────────────────
    def _build_trajectory_section(self):
        f = self.scroll_frame
        SectionHeader(f, "TRAJECTORY SETTINGS", ACCENT2).pack(fill="x", padx=8, pady=(10, 2))
        card = tk.Frame(f, bg=BG_CARD, bd=0, pady=8, padx=8)
        card.pack(fill="x", padx=8, pady=4)

        self.q_target_1 = LabelEntry(card, "Góc đích khớp 1 (q_target):", self.cfg["q_target_1"], "rad")
        self.q_target_1.pack(fill="x", pady=3)
        self.q_target_2 = LabelEntry(card, "Góc đích khớp 2 (q_target):", self.cfg["q_target_2"], "rad")
        self.q_target_2.pack(fill="x", pady=3)
        self.traj_tf = LabelEntry(card, "Thời gian đến đích (T_f):", self.cfg["trajectory_tf"], "s")
        self.traj_tf.pack(fill="x", pady=3)

    # ──────────────────────────────────────────────
    # SECTION: DISTURBANCE
    # ──────────────────────────────────────────────
    def _build_disturbance_section(self):
        f = self.scroll_frame
        SectionHeader(f, "DISTURBANCE SETTINGS", YELLOW).pack(fill="x", padx=8, pady=(10, 2))
        card = tk.Frame(f, bg=BG_CARD, bd=0, pady=8, padx=8)
        card.pack(fill="x", padx=8, pady=4)

        # Toggle
        toggle_row = tk.Frame(card, bg=BG_CARD)
        toggle_row.pack(fill="x", pady=4)
        tk.Label(toggle_row, text="Nhiễu ngoại lực:", font=FONT_LABEL, bg=BG_CARD, fg=TEXT_WHITE, width=24, anchor="w").pack(side="left", padx=6)
        self.dist_var = tk.BooleanVar(value=True)
        tk.Checkbutton(toggle_row, text="Bật", variable=self.dist_var,
                       font=FONT_LABEL, bg=BG_CARD, fg=YELLOW,
                       selectcolor=BG_DARK, activebackground=BG_CARD).pack(side="left")

        # Sin disturbance header
        tk.Label(card, text="  ▸ Nhiễu hình sin (Sinusoidal)", font=FONT_HEADER,
                 bg=BG_CARD, fg=ACCENT2, anchor="w").pack(fill="x", padx=6, pady=(8, 2))
        row1 = tk.Frame(card, bg=BG_CARD)
        row1.pack(fill="x")
        self.sin_amp_1  = LabelEntry(row1, "Biên độ khớp 1:", self.cfg["sin_amp_1"], "Nm")
        self.sin_amp_1.pack(side="left", padx=4)
        self.sin_freq_1 = LabelEntry(row1, "Tần số khớp 1:", self.cfg["sin_freq_1"], "rad/s")
        self.sin_freq_1.pack(side="left", padx=4)
        row2 = tk.Frame(card, bg=BG_CARD)
        row2.pack(fill="x", pady=3)
        self.sin_amp_2  = LabelEntry(row2, "Biên độ khớp 2:", self.cfg["sin_amp_2"], "Nm")
        self.sin_amp_2.pack(side="left", padx=4)
        self.sin_freq_2 = LabelEntry(row2, "Tần số khớp 2:", self.cfg["sin_freq_2"], "rad/s")
        self.sin_freq_2.pack(side="left", padx=4)

        # Pulse disturbance header
        tk.Label(card, text="  ▸ Xung nhiễu đột ngột (Pulse Disturbance)", font=FONT_HEADER,
                 bg=BG_CARD, fg=YELLOW, anchor="w").pack(fill="x", padx=6, pady=(8, 2))
        row3 = tk.Frame(card, bg=BG_CARD)
        row3.pack(fill="x")
        self.pulse_start = LabelEntry(row3, "Thời điểm bắt đầu:", self.cfg["pulse_start"], "s")
        self.pulse_start.pack(side="left", padx=4)
        self.pulse_end   = LabelEntry(row3, "Thời điểm kết thúc:", self.cfg["pulse_end"], "s")
        self.pulse_end.pack(side="left", padx=4)
        row4 = tk.Frame(card, bg=BG_CARD)
        row4.pack(fill="x", pady=3)
        self.pulse_amp_1 = LabelEntry(row4, "Biên độ xung khớp 1:", self.cfg["pulse_amp_1"], "Nm")
        self.pulse_amp_1.pack(side="left", padx=4)
        self.pulse_amp_2 = LabelEntry(row4, "Biên độ xung khớp 2:", self.cfg["pulse_amp_2"], "Nm")
        self.pulse_amp_2.pack(side="left", padx=4)

    # ──────────────────────────────────────────────
    # SECTION: SMC
    # ──────────────────────────────────────────────
    def _build_smc_section(self):
        f = self.scroll_frame
        SectionHeader(f, "SMC CONTROLLER PARAMETERS", GREEN).pack(fill="x", padx=8, pady=(10, 2))
        card = tk.Frame(f, bg=BG_CARD, bd=0, pady=8, padx=8)
        card.pack(fill="x", padx=8, pady=4)

        # use_sat toggle
        sat_row = tk.Frame(card, bg=BG_CARD)
        sat_row.pack(fill="x", pady=4)
        tk.Label(sat_row, text="Hàm chuyển mạch:", font=FONT_LABEL, bg=BG_CARD, fg=TEXT_WHITE, width=24, anchor="w").pack(side="left", padx=6)
        self.smc_sat_var = tk.BooleanVar(value=True)
        tk.Radiobutton(sat_row, text="Sat (chống chattering)", variable=self.smc_sat_var, value=True,
                       font=FONT_LABEL, bg=BG_CARD, fg=GREEN, selectcolor=BG_DARK,
                       activebackground=BG_CARD).pack(side="left", padx=6)
        tk.Radiobutton(sat_row, text="Sign (chattering mạnh)", variable=self.smc_sat_var, value=False,
                       font=FONT_LABEL, bg=BG_CARD, fg=YELLOW, selectcolor=BG_DARK,
                       activebackground=BG_CARD).pack(side="left", padx=6)

        # Lambda
        tk.Label(card, text="  ▸ Ma trận Lambda (mặt trượt)", font=FONT_HEADER, bg=BG_CARD, fg=ACCENT2, anchor="w").pack(fill="x", padx=6, pady=(6, 2))
        row_l = tk.Frame(card, bg=BG_CARD); row_l.pack(fill="x")
        self.smc_lambda_1 = LabelEntry(row_l, "Lambda khớp 1:", self.cfg["smc_lambda_1"])
        self.smc_lambda_1.pack(side="left", padx=4)
        self.smc_lambda_2 = LabelEntry(row_l, "Lambda khớp 2:", self.cfg["smc_lambda_2"])
        self.smc_lambda_2.pack(side="left", padx=4)

        # K
        tk.Label(card, text="  ▸ Ma trận K (độ lợi tỷ lệ)", font=FONT_HEADER, bg=BG_CARD, fg=ACCENT2, anchor="w").pack(fill="x", padx=6, pady=(6, 2))
        row_k = tk.Frame(card, bg=BG_CARD); row_k.pack(fill="x")
        self.smc_k_1 = LabelEntry(row_k, "K khớp 1:", self.cfg["smc_k_1"])
        self.smc_k_1.pack(side="left", padx=4)
        self.smc_k_2 = LabelEntry(row_k, "K khớp 2:", self.cfg["smc_k_2"])
        self.smc_k_2.pack(side="left", padx=4)

        # Eta
        tk.Label(card, text="  ▸ Ma trận Eta (độ lợi chống nhiễu)", font=FONT_HEADER, bg=BG_CARD, fg=ACCENT2, anchor="w").pack(fill="x", padx=6, pady=(6, 2))
        row_e = tk.Frame(card, bg=BG_CARD); row_e.pack(fill="x")
        self.smc_eta_1 = LabelEntry(row_e, "Eta khớp 1:", self.cfg["smc_eta_1"])
        self.smc_eta_1.pack(side="left", padx=4)
        self.smc_eta_2 = LabelEntry(row_e, "Eta khớp 2:", self.cfg["smc_eta_2"])
        self.smc_eta_2.pack(side="left", padx=4)

        # Phi
        row_p = tk.Frame(card, bg=BG_CARD); row_p.pack(fill="x", pady=4)
        self.smc_phi = LabelEntry(row_p, "Phi (bề dày lớp biên):", self.cfg["smc_phi"])
        self.smc_phi.pack(side="left", padx=4)

    # ──────────────────────────────────────────────
    # SECTION: PID
    # ──────────────────────────────────────────────
    def _build_pid_section(self):
        f = self.scroll_frame
        SectionHeader(f, "PID CONTROLLER PARAMETERS", ACCENT).pack(fill="x", padx=8, pady=(10, 2))
        card = tk.Frame(f, bg=BG_CARD, bd=0, pady=8, padx=8)
        card.pack(fill="x", padx=8, pady=(4, 16))

        # Kp
        tk.Label(card, text="  ▸ Kp", font=FONT_HEADER, bg=BG_CARD, fg=ACCENT2, anchor="w").pack(fill="x", padx=6, pady=(4, 2))
        row_p = tk.Frame(card, bg=BG_CARD); row_p.pack(fill="x")
        self.pid_kp_1 = LabelEntry(row_p, "Kp khớp 1:", self.cfg["pid_kp_1"])
        self.pid_kp_1.pack(side="left", padx=4)
        self.pid_kp_2 = LabelEntry(row_p, "Kp khớp 2:", self.cfg["pid_kp_2"])
        self.pid_kp_2.pack(side="left", padx=4)

        # Ki
        tk.Label(card, text="  ▸ Ki", font=FONT_HEADER, bg=BG_CARD, fg=ACCENT2, anchor="w").pack(fill="x", padx=6, pady=(6, 2))
        row_i = tk.Frame(card, bg=BG_CARD); row_i.pack(fill="x")
        self.pid_ki_1 = LabelEntry(row_i, "Ki khớp 1:", self.cfg["pid_ki_1"])
        self.pid_ki_1.pack(side="left", padx=4)
        self.pid_ki_2 = LabelEntry(row_i, "Ki khớp 2:", self.cfg["pid_ki_2"])
        self.pid_ki_2.pack(side="left", padx=4)

        # Kd
        tk.Label(card, text="  ▸ Kd", font=FONT_HEADER, bg=BG_CARD, fg=ACCENT2, anchor="w").pack(fill="x", padx=6, pady=(6, 2))
        row_d = tk.Frame(card, bg=BG_CARD); row_d.pack(fill="x")
        self.pid_kd_1 = LabelEntry(row_d, "Kd khớp 1:", self.cfg["pid_kd_1"])
        self.pid_kd_1.pack(side="left", padx=4)
        self.pid_kd_2 = LabelEntry(row_d, "Kd khớp 2:", self.cfg["pid_kd_2"])
        self.pid_kd_2.pack(side="left", padx=4)

        # Windup
        row_w = tk.Frame(card, bg=BG_CARD); row_w.pack(fill="x", pady=4)
        self.pid_windup = LabelEntry(row_w, "Anti-Windup giới hạn:", self.cfg["pid_windup"])
        self.pid_windup.pack(side="left", padx=4)

    # ──────────────────────────────────────────────
    # LOAD VALUES FROM CONFIG
    # ──────────────────────────────────────────────
    def _load_values(self):
        c = self.cfg
        self.mode_var.set(c["mode"])
        self.dist_var.set(c["add_disturbance"])
        self.duration_entry.set(c["duration"])
        self.q_target_1.set(c["q_target_1"])
        self.q_target_2.set(c["q_target_2"])
        self.traj_tf.set(c["trajectory_tf"])
        self.sin_amp_1.set(c["sin_amp_1"])
        self.sin_freq_1.set(c["sin_freq_1"])
        self.sin_amp_2.set(c["sin_amp_2"])
        self.sin_freq_2.set(c["sin_freq_2"])
        self.pulse_start.set(c["pulse_start"])
        self.pulse_end.set(c["pulse_end"])
        self.pulse_amp_1.set(c["pulse_amp_1"])
        self.pulse_amp_2.set(c["pulse_amp_2"])
        self.smc_lambda_1.set(c["smc_lambda_1"])
        self.smc_lambda_2.set(c["smc_lambda_2"])
        self.smc_k_1.set(c["smc_k_1"])
        self.smc_k_2.set(c["smc_k_2"])
        self.smc_eta_1.set(c["smc_eta_1"])
        self.smc_eta_2.set(c["smc_eta_2"])
        self.smc_phi.set(c["smc_phi"])
        self.smc_sat_var.set(c["smc_use_sat"])
        self.pid_kp_1.set(c["pid_kp_1"])
        self.pid_kp_2.set(c["pid_kp_2"])
        self.pid_ki_1.set(c["pid_ki_1"])
        self.pid_ki_2.set(c["pid_ki_2"])
        self.pid_kd_1.set(c["pid_kd_1"])
        self.pid_kd_2.set(c["pid_kd_2"])
        self.pid_windup.set(c["pid_windup"])

    # ──────────────────────────────────────────────
    # COLLECT VALUES → DICT
    # ──────────────────────────────────────────────
    def _collect(self):
        try:
            return {
                "mode":           int(self.mode_var.get()),
                "add_disturbance": bool(self.dist_var.get()),
                "duration":        float(self.duration_entry.get()),
                "q_target_1":      float(self.q_target_1.get()),
                "q_target_2":      float(self.q_target_2.get()),
                "trajectory_tf":   float(self.traj_tf.get()),
                "sin_amp_1":       float(self.sin_amp_1.get()),
                "sin_freq_1":      float(self.sin_freq_1.get()),
                "sin_amp_2":       float(self.sin_amp_2.get()),
                "sin_freq_2":      float(self.sin_freq_2.get()),
                "pulse_start":     float(self.pulse_start.get()),
                "pulse_end":       float(self.pulse_end.get()),
                "pulse_amp_1":     float(self.pulse_amp_1.get()),
                "pulse_amp_2":     float(self.pulse_amp_2.get()),
                "smc_lambda_1":    float(self.smc_lambda_1.get()),
                "smc_lambda_2":    float(self.smc_lambda_2.get()),
                "smc_k_1":         float(self.smc_k_1.get()),
                "smc_k_2":         float(self.smc_k_2.get()),
                "smc_eta_1":       float(self.smc_eta_1.get()),
                "smc_eta_2":       float(self.smc_eta_2.get()),
                "smc_phi":         float(self.smc_phi.get()),
                "smc_use_sat":     bool(self.smc_sat_var.get()),
                "pid_kp_1":        float(self.pid_kp_1.get()),
                "pid_kp_2":        float(self.pid_kp_2.get()),
                "pid_ki_1":        float(self.pid_ki_1.get()),
                "pid_ki_2":        float(self.pid_ki_2.get()),
                "pid_kd_1":        float(self.pid_kd_1.get()),
                "pid_kd_2":        float(self.pid_kd_2.get()),
                "pid_windup":      float(self.pid_windup.get()),
            }
        except ValueError as e:
            messagebox.showerror("Lỗi giá trị", f"Giá trị không hợp lệ:\n{e}")
            return None

    # ──────────────────────────────────────────────
    # ACTIONS
    # ──────────────────────────────────────────────
    def _save(self):
        cfg = self._collect()
        if cfg is None:
            return
        save_config(cfg)
        self.cfg = cfg
        messagebox.showinfo("Thành công ✅",
                            "Cấu hình đã được lưu vào sim_config.json\n"
                            "Chạy simulate_robot.py để áp dụng.")

    def _reset(self):
        if messagebox.askyesno("Reset", "Bạn có chắc muốn reset về giá trị mặc định?"):
            self.cfg = DEFAULT_CONFIG.copy()
            self._load_values()

    def _run_sim(self):
        cfg = self._collect()
        if cfg is None:
            return
        save_config(cfg)
        import subprocess, sys
        script_path = os.path.join(os.path.dirname(__file__), "simulate_robot.py")
        subprocess.Popen([sys.executable, script_path],
                         cwd=os.path.dirname(__file__))
        messagebox.showinfo("Đã khởi chạy ▶",
                            "simulate_robot.py đang chạy trong cửa sổ mới.\n"
                            "Đóng MuJoCo viewer để xem biểu đồ kết quả.")


# ──────────────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app = SimConfigGUI(root)
    root.mainloop()
