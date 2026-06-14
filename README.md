# 🏗️ 3D Concrete Printing Digital Twin Demo

This repository contains a lightweight Digital Twin architecture designed for autonomous 3D concrete printing robotic systems. It bridges the gap between high-level path planning, 3D simulation, and low-level real-time PLC control.

## 🌟 System Architecture

The architecture relies on a "Trinity" integration:
1. **RoboDK (Physical Kinematics):** Simulates a 6-axis heavy-duty industrial robot (e.g., ABB/KUKA) equipped with a concrete extruder (e.g., CEAD). Handles singularity avoidance and linear interpolation (`MoveL`).
2. **Python (Middle-layer Orchestration):** Generates parametric 3D printing trajectories (G-code equivalent) and acts as the master scheduler.
3. **Beckhoff TwinCAT 3 (Soft-PLC Kernel):** Receives real-time absolute spatial coordinates (X, Y, Z) and coupled pump speeds via the ADS protocol for strict hardware execution.

## 🚀 Quick Start

### 1. Prerequisites
- Python 3.7+
- RoboDK software installed
- TwinCAT 3 runtime enabled (Target NetID properly configured)

### 2. Installation
Clone the repository and install the required packages:

```bash
git clone [https://github.com/Arslanpro/Vertico_Digital_Twin.git](https://github.com/Arslanpro/Vertico_Digital_Twin.git)
cd Vertico_Digital_Twin
pip install -r requirements.txt
```

### 3. Execution
1. Open the `Vertico_Digital_Twin.rdk` station file in RoboDK.
2. Ensure your TwinCAT soft-PLC is in `RUN` mode.
3. Update the `AMS_NET_ID` in the script to match your local PLC route.
4. Run the orchestration script:

```bash
python digital_twin_demo.py
```

## 🎥 Demonstration
Upon execution, the script will command the robotic arm to move out of the singularity (Home Pose) and begin executing a smooth spiral trajectory. The instantaneous XYZ coordinates and dynamic pump speed will be asynchronously pushed to the TwinCAT memory registers.
