# SMC Robot Arm Simulation 🤖

A 2-DOF (Degrees of Freedom) robotic arm control simulation utilizing **Sliding Mode Control (SMC)** and **PID** within the **MuJoCo** physics engine. It comes with a customized "Cyberpunk/Sci-Fi" style GUI configuration panel for visual and intuitive parameter tuning.

## Key Features
- Performance comparison between PID and SMC controllers under external disturbances (sinusoidal friction, collision pulses).
- Accurate 2-DOF robot dynamics simulation using inertia matrix data extracted directly from CAD designs.
- A visual GUI Control Panel to easily configure modes, target trajectories, disturbance parameters, and controller hyperparameters.
- Comprehensive resulting charts: Reference vs Actual Trajectory, Tracking Error, and Control Input Torque.

---

## Setup & Installation Guide

### 1. Prerequisites
- OS: Windows, macOS, or Linux.
- **Python 3.9 or higher** (Python 3.10 or 3.11 is highly recommended).
  > **Note for beginners:** If you don't have Python installed, download it from [python.org/downloads](https://www.python.org/downloads/). When running the installer on Windows, make sure to **CHECK THE BOX "Add python.exe to PATH"** before clicking Install.

### 2. Clone the Repository
Open your Terminal (or Powershell/Command Prompt) and run the following command:
```bash
git clone https://github.com/your-username/SMC_Robot_Arm.git
cd SMC_Robot_Arm
```

### 3. Create a Virtual Environment
It is highly recommended to use a virtual environment to avoid dependency conflicts with other projects.
```bash
# Create a virtual environment named mujoco_env
python -m venv mujoco_env

# Activate the environment (Windows)
.\mujoco_env\Scripts\activate

# Activate the environment (macOS/Linux)
source mujoco_env/bin/activate
```
*(If activated successfully, you will see `(mujoco_env)` at the beginning of your command prompt).*

### 4. Install Dependencies
Ensure you are in the activated virtual environment, then install the required packages:
```bash
pip install -r requirements.txt
```

---

## Usage Guide

To launch the Orbital Command Center (Config Panel), run the following command:
```bash
python sim_gui.py
```

1. **Setup:** Adjust the control parameters (PID/SMC), toggle external disturbances, or configure target trajectories directly on the interface.
2. **Save:** Click the **💾 SAVE** button to save the configuration into `sim_config.json`.
3. **Launch:** Click the **🚀 RUN** button to begin the simulation. The MuJoCo 3D window will open, and once the simulation completes, matplotlib report charts will pop up automatically.

---
*Designed by: Nhat_Minh*
