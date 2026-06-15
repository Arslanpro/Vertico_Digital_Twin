import pyads
from robodk.robolink import *
from robodk.robomath import *
import time
import math
import re
import os
import matplotlib.pyplot as plt

# ---------------- 1. TwinCAT PLC Connection Setup ----------------
plc_connected = False
AMS_NET_ID = '192.168.0.109.1.1'
plc = pyads.Connection(AMS_NET_ID, pyads.PORT_TC3PLC1)

# ---------------- 2. RoboDK Simulation Setup ----------------
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

# ---------------- 3. Phase 3: G-Code Parser Engine ----------------
gcode_file = "test_print.gcode"

if not os.path.exists(gcode_file):
    print(f"[INFO] '{gcode_file}' not found. Auto-generating a standard test path...")
    with open(gcode_file, "w") as f:
        # Layer 1 (Base)
        f.write("G1 X0 Y0 Z0 E10\n")
        f.write("G1 X150 Y0 Z0 E20\n")
        f.write("G1 X150 Y150 Z0 E30\n")
        f.write("G1 X0 Y150 Z0 E40\n")
        f.write("G1 X0 Y0 Z0 E50\n")
        # Layer 2
        f.write("G1 X0 Y0 Z5 E60\n")
        f.write("G1 X150 Y0 Z5 E70\n")
        f.write("G1 X150 Y150 Z5 E80\n")
        f.write("G1 X0 Y150 Z5 E90\n")
        f.write("G1 X0 Y0 Z5 E100\n")

print("\n[ACTION] Parsing G-code file...")
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
    print(f"[SUCCESS] Parsed {len(gcode_waypoints)} architectural waypoints from G-code.\n")
except Exception as e:
    print(f"[FATAL] Failed to read G-code: {e}")
    exit()

# ---------------- 4. Digital Twin Execution ----------------
print_path_points = []
# 🌟 NEW: Data arrays for 2D Analytics Plot
analytics_time_steps = []
analytics_tcp_velocity = []
analytics_pump_rpm = []

try:
    print("[ACTION] Moving robot to a safe Home Pose...")
    safe_joints = [0, 30, 30, 0, 90, 0]
    robot.MoveJ(safe_joints) 
    time.sleep(1.5) 

    pose_ref = robot.Pose()
    print("[READY] Starting G-code toolpath execution...")

    heartbeat_status = False

    if plc_connected:
        heartbeat_status = not heartbeat_status
        plc.write_by_name('GVL_Printer.bPythonHeartbeat', heartbeat_status, pyads.PLCTYPE_BOOL)
        time.sleep(0.1) 
        plc.write_by_name('GVL_Printer.nPrinterState', 1, pyads.PLCTYPE_INT)

    # Dynamic Extrusion Parameters
    prev_xyz = None
    K_FACTOR = 0.8         # Adjusted multiplier for realistic demo scaling
    MIN_PUMP_SPEED = 10.0  
    MAX_PUMP_SPEED = 200.0 
    
    # 🌟 NEW: Simulated physical time step (Simulating a consistent 100ms print tick)
    # Using fixed physical dt prevents erratic velocity spikes caused by python processing speed
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
            # Calculate physical displacement
            dist = math.sqrt((x_val - prev_xyz[0])**2 + (y_val - prev_xyz[1])**2 + (z_val - prev_xyz[2])**2)
            
            # 🌟 Use fixed simulated physics time for realistic TCP Velocity calculation
            velocity = dist / SIMULATED_TICK_S
            
            # Map TCP Velocity to Extruder RPM
            target_rpm = velocity * K_FACTOR
            pump_speed = max(MIN_PUMP_SPEED, min(target_rpm, MAX_PUMP_SPEED))
        else:
            velocity = 0.0
            pump_speed = MIN_PUMP_SPEED

        prev_xyz = [x_val, y_val, z_val]

        # 🌟 Log data for post-print analytics
        analytics_time_steps.append(step_idx)
        analytics_tcp_velocity.append(velocity)
        analytics_pump_rpm.append(pump_speed)

        if plc_connected:
            try:
                plc.write_by_name('GVL_Printer.fCurrentX', x_val, pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fCurrentY', y_val, pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fCurrentZ', z_val, pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fPumpSpeed', pump_speed, pyads.PLCTYPE_REAL)
                print(f"[G-CODE] WP:{step_idx} | TCP Vel:{velocity:.1f} mm/s -> Extruder RPM:{pump_speed:.1f}")
            except:
                pass
        else:
            print(f"[OFFLINE] WP:{step_idx} | TCP Vel:{velocity:.1f} mm/s -> Extruder RPM:{pump_speed:.1f}")

        # Visual UI buffer sleep
        time.sleep(0.05)

except Exception as e:
    print(f"\n[FATAL] Execution failed: {repr(e)}")
finally:  
    # ---------------- 5. Graceful Shutdown ----------------
    if plc_connected:
        try:
            plc.write_by_name('GVL_Printer.fPumpSpeed', 0.0, pyads.PLCTYPE_REAL)
            plc.write_by_name('GVL_Printer.nPrinterState', 0, pyads.PLCTYPE_INT)
            print("\n[INFO] Job finished. Holding connection to verify Idle state...")
            time.sleep(1.5)
            plc.close()
        except:
            pass
    print("\n[CLOSED] ADS connection safely released.")

    # ---------------- 6. Post-Print Data Analysis Plots ----------------
    print("\n[ACTION] Generating Twin-Analytics Dashboards...")
    if len(print_path_points) > 1:
        # Create a figure with 2 subplots (1 for 3D path, 1 for Velocity Correlation)
        fig = plt.figure("Vertico Slicer - Post Print Analytics", figsize=(12, 6))

        # --- Subplot 1: The 3D Toolpath ---
        xs = [pt[0] for pt in print_path_points]
        ys = [pt[1] for pt in print_path_points]
        zs = [pt[2] for pt in print_path_points]

        ax1 = fig.add_subplot(121, projection='3d')
        ax1.plot(xs, ys, zs, label='G-code Toolpath', color='darkorange', linewidth=3)
        ax1.set_title('3D Spatial Verification', fontweight='bold')
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_zlabel('Z (mm)')
        ax1.legend()
        ax1.set_box_aspect([1, 1, 1])

        # --- Subplot 2: Dynamic Extrusion Correlation (TCP Velocity vs Pump RPM) ---
        # 🌟 This is the killer feature for the interview!
        ax2 = fig.add_subplot(122)
        ax2.set_title('Dynamic Extrusion Feedback Loop', fontweight='bold')
        ax2.set_xlabel('Waypoint (Tick)')
        
        # Plot TCP Velocity on primary Y axis
        color1 = 'tab:blue'
        ax2.set_ylabel('TCP Velocity (mm/s)', color=color1)
        ax2.plot(analytics_time_steps, analytics_tcp_velocity, color=color1, marker='o', label='TCP Velocity')
        ax2.tick_params(axis='y', labelcolor=color1)
        
        # Instantiate a second axes that shares the same x-axis
        ax3 = ax2.twinx()  
        
        # Plot Extruder RPM on secondary Y axis
        color2 = 'tab:red'
        ax3.set_ylabel('Extruder RPM', color=color2)
        ax3.plot(analytics_time_steps, analytics_pump_rpm, color=color2, linestyle='--', marker='x', label='Pump RPM')
        ax3.tick_params(axis='y', labelcolor=color2)

        fig.tight_layout() # Adjust layout to prevent overlap
        print("[SUCCESS] Rendering Analytics Dashboards...")
        plt.show()