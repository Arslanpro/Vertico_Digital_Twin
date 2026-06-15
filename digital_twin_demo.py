import pyads
from robodk.robolink import *
from robodk.robomath import *
import time
import math
import re
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog  # 🌟 引入弹窗选择器

# ---------------- 1. UI: Select G-code File ----------------
# 隐藏多余的 tk 主窗口，只保留选择文件的弹窗
root = tk.Tk()
root.withdraw() 
print("[ACTION] Awaiting user to select a G-code file...")

gcode_file = filedialog.askopenfilename(
    title="Select Architectural G-code File",
    filetypes=[("G-code Files", "*.gcode"), ("All Files", "*.*")]
)

# 如果用户点了取消，直接安全退出程序
if not gcode_file:
    print("[INFO] No file selected. Execution cancelled.")
    exit()

print(f"[SUCCESS] Selected Toolpath File: {gcode_file.split('/')[-1]}\n")

# ---------------- 2. TwinCAT PLC Connection Setup ----------------
plc_connected = False
AMS_NET_ID = '192.168.0.109.1.1'
plc = pyads.Connection(AMS_NET_ID, pyads.PORT_TC3PLC1)

# ---------------- 3. RoboDK Simulation Setup ----------------
RDK = Robolink()
robot = RDK.Item('', ITEM_TYPE_ROBOT) 

if not robot.Valid():
    print("[ERROR] Robot model not found in RoboDK. Please ensure the station is loaded.")
    exit()

try:
    plc.open()
    plc.write_by_name('GVL_Printer.bSystemReady', True, pyads.PLCTYPE_BOOL)
    plc_connected = True
    print("[SUCCESS] Connected to TwinCAT PLC Real-time Kernel.\n")
except Exception as e:
    print("[WARNING] TwinCAT Kernel access blocked (IT policy restrictions).")
    print("[INFO] Fallback activated: Running in 'Offline Simulation Only' mode...\n")
    plc_connected = False

# ---------------- 4. Phase 3: G-Code Parser ----------------
print("\n[ACTION] Parsing architectural G-code file...")
gcode_waypoints = []
try:
    with open(gcode_file, "r") as file:
        for line in file:
            line = line.strip()
            if line.startswith("G1"):
                x_match = re.search(r'X([\d\.\-]+)', line)
                y_match = re.search(r'Y([\d\.\-]+)', line)
                z_match = re.search(r'Z([\d\.\-]+)', line)
                
                if x_match and y_match and z_match:
                    gcode_waypoints.append([
                        float(x_match.group(1)), 
                        float(y_match.group(1)), 
                        float(z_match.group(1))
                    ])
    print(f"[SUCCESS] Sliced into {len(gcode_waypoints)} dynamic waypoints.\n")
except Exception as e:
    print(f"[FATAL] Failed to read G-code: {e}")
    exit()

# ---------------- 5. Digital Twin Execution (RoboDK + TwinCAT) ----------------
print_path_points = []
analytics_time_steps = []
analytics_tcp_velocity = []
analytics_pump_rpm = []

try:
    print("[ACTION] Moving robot to a safe Home Pose...")
    safe_joints = [0, 30, 30, 0, 90, 0]
    robot.MoveJ(safe_joints) 
    time.sleep(1.5) 

    pose_ref = robot.Pose()
    print("[READY] Executing 3D Concrete Print. Watch RoboDK for the live simulation...")

    heartbeat_status = False

    if plc_connected:
        heartbeat_status = not heartbeat_status
        plc.write_by_name('GVL_Printer.bPythonHeartbeat', heartbeat_status, pyads.PLCTYPE_BOOL)
        time.sleep(0.1) 
        plc.write_by_name('GVL_Printer.nPrinterState', 1, pyads.PLCTYPE_INT)

    prev_xyz = None
    K_FACTOR = 0.6         
    MIN_PUMP_SPEED = 15.0  
    MAX_PUMP_SPEED = 250.0 
    SIMULATED_TICK_S = 0.1 

    for step_idx, wp in enumerate(gcode_waypoints):
        if plc_connected:
            heartbeat_status = not heartbeat_status
            plc.write_by_name('GVL_Printer.bPythonHeartbeat', heartbeat_status, pyads.PLCTYPE_BOOL)
        
        target_pose = transl(wp[0], wp[1], wp[2]) * pose_ref
        robot.MoveL(target_pose)
        
        current_xyz = target_pose.Pos()
        x_val, y_val, z_val = float(current_xyz[0]), float(current_xyz[1]), float(current_xyz[2])

        print_path_points.append([x_val, y_val, z_val])
        
        # Velocity-Dependent Extrusion Logic
        if prev_xyz is not None:
            dist = math.sqrt((x_val - prev_xyz[0])**2 + (y_val - prev_xyz[1])**2 + (z_val - prev_xyz[2])**2)
            velocity = dist / SIMULATED_TICK_S
            target_rpm = velocity * K_FACTOR
            pump_speed = max(MIN_PUMP_SPEED, min(target_rpm, MAX_PUMP_SPEED))
        else:
            velocity = 0.0
            pump_speed = MIN_PUMP_SPEED

        prev_xyz = [x_val, y_val, z_val]

        analytics_time_steps.append(step_idx)
        analytics_tcp_velocity.append(velocity)
        analytics_pump_rpm.append(pump_speed)

        # Output every 5th step to keep terminal clean
        if step_idx % 5 == 0:
            if plc_connected:
                try:
                    plc.write_by_name('GVL_Printer.fCurrentX', x_val, pyads.PLCTYPE_REAL)
                    plc.write_by_name('GVL_Printer.fCurrentY', y_val, pyads.PLCTYPE_REAL)
                    plc.write_by_name('GVL_Printer.fCurrentZ', z_val, pyads.PLCTYPE_REAL)
                    plc.write_by_name('GVL_Printer.fPumpSpeed', pump_speed, pyads.PLCTYPE_REAL)
                    print(f"[PRINTING] Z:{z_val:.1f} | WP:{step_idx} | TCP Vel:{velocity:.1f} mm/s -> RPM:{pump_speed:.1f}")
                except:
                    pass
            else:
                print(f"[OFFLINE] Z:{z_val:.1f} | WP:{step_idx} | TCP Vel:{velocity:.1f} mm/s -> RPM:{pump_speed:.1f}")

        time.sleep(0.02)

except Exception as e:
    print(f"\n[FATAL] Execution failed: {repr(e)}")
finally:  
    # ---------------- 6. Graceful Shutdown ----------------
    if plc_connected:
        try:
            plc.write_by_name('GVL_Printer.fPumpSpeed', 0.0, pyads.PLCTYPE_REAL)
            plc.write_by_name('GVL_Printer.nPrinterState', 0, pyads.PLCTYPE_INT)
            print("\n[INFO] Print finished. Holding connection to verify Idle state...")
            time.sleep(1.5)
            plc.close()
        except:
            pass
    print("\n[CLOSED] ADS connection safely released.")

    # ---------------- 7. Post-Print Data Analysis Plots ----------------
    print("\n[ACTION] Generating Twin-Analytics Dashboards...")
    if len(print_path_points) > 1:
        fig = plt.figure("Vertico Slicer - Post Print Analytics", figsize=(14, 7))

        xs = [pt[0] for pt in print_path_points]
        ys = [pt[1] for pt in print_path_points]
        zs = [pt[2] for pt in print_path_points]

        ax1 = fig.add_subplot(121, projection='3d')
        ax1.plot(xs, ys, zs, label='G-code Toolpath', color='darkorange', linewidth=2)
        ax1.set_title('3D Structural Verification', fontweight='bold')
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_zlabel('Z (mm)')
        ax1.legend()
        ax1.set_box_aspect([1, 1, 1.2])

        ax2 = fig.add_subplot(122)
        ax2.set_title('Velocity-Dependent Extrusion (Zoomed: Last 150 Waypoints)', fontweight='bold')
        ax2.set_xlabel('Waypoint (Tick)')
        
        zoom_range = min(150, len(analytics_time_steps))
        display_steps = analytics_time_steps[-zoom_range:]
        display_vel = analytics_tcp_velocity[-zoom_range:]
        display_rpm = analytics_pump_rpm[-zoom_range:]

        color1 = 'tab:blue'
        ax2.set_ylabel('TCP Velocity (mm/s)', color=color1)
        ax2.plot(display_steps, display_vel, color=color1, alpha=0.8, linewidth=2, label='TCP Velocity')
        ax2.tick_params(axis='y', labelcolor=color1)
        
        ax3 = ax2.twinx()  
        color2 = 'tab:red'
        ax3.set_ylabel('Extruder RPM', color=color2)
        ax3.scatter(display_steps, display_rpm, color=color2, marker='x', s=30, alpha=0.8, label='Pump RPM')
        ax3.tick_params(axis='y', labelcolor=color2)

        fig.tight_layout()
        print("[SUCCESS] Rendering Dashboards...")
        plt.show()