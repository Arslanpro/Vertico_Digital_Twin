import pyads
from robodk.robolink import *
from robodk.robomath import *
import time
import math
import matplotlib.pyplot as plt  # Used for 3D path visualization and analysis

# ---------------- 1. TwinCAT PLC Connection Setup ----------------
plc_connected = False
AMS_NET_ID = '192.168.0.109.1.1'
# Port 851 is the standard default for TwinCAT 3 PLC runtimes
plc = pyads.Connection(AMS_NET_ID, pyads.PORT_TC3PLC1)

# ---------------- 2. RoboDK Simulation Setup ----------------
RDK = Robolink()
robot = RDK.Item('', ITEM_TYPE_ROBOT) 

if not robot.Valid():
    print("[ERROR] Robot model not found in RoboDK. Please ensure the station is loaded.")
    exit()

# Try establishing ADS connection to the soft-PLC
try:
    plc.open()
    # Signal the PLC that the orchestrator is online and ready
    plc.write_by_name('GVL_Printer.bSystemReady', True, pyads.PLCTYPE_BOOL)
    plc_connected = True
    print("[SUCCESS] Connected to TwinCAT PLC Real-time Kernel.\n")
except Exception as e:
    print("[WARNING] TwinCAT Kernel access blocked (IT policy restrictions).")
    print("[INFO] Fallback activated: Running in 'Offline Simulation Only' mode...\n")
    plc_connected = False

# List to buffer trajectory points for post-print analysis
print_path_points = []

try:
    # --- Kinematic Singularity Avoidance ---
    print("[ACTION] Moving robot to a safe Home Pose to avoid singularities...")
    safe_joints = [0, 30, 30, 0, 90, 0]
    robot.MoveJ(safe_joints) 
    time.sleep(1.5) 

    # Define the reference frame based on the initial safe pose
    pose_ref = robot.Pose()
    print("[READY] Pose calibration complete. Starting print execution...")

    heartbeat_status = False

    # --- Handshake Initialization ---
    if plc_connected:
        # Toggle heartbeat to clear any initial Watchdog alarms
        heartbeat_status = not heartbeat_status
        plc.write_by_name('GVL_Printer.bPythonHeartbeat', heartbeat_status, pyads.PLCTYPE_BOOL)
        time.sleep(0.1) 
        # Set PLC state machine to 'Printing' (State 1)
        plc.write_by_name('GVL_Printer.nPrinterState', 1, pyads.PLCTYPE_INT)

    # ---------------- 3. Main Digital Twin Loop ----------------
    for i in range(50):
        # Heartbeat pulse to keep the PLC Watchdog satisfied
        if plc_connected:
            heartbeat_status = not heartbeat_status
            plc.write_by_name('GVL_Printer.bPythonHeartbeat', heartbeat_status, pyads.PLCTYPE_BOOL)

        ang = math.radians(i * 10)
        
        # Calculate parametric spiral path
        dx = 150 * math.cos(ang) - 150  
        dy = 150 * math.sin(ang)
        dz = i * 1.5 # Incremental layer height simulation
        
        target_pose = transl(dx, dy, dz) * pose_ref
        robot.MoveL(target_pose)
        current_xyz = target_pose.Pos()
        
        # Explicitly cast Mat objects to float for clean data logging
        x_val = float(current_xyz[0])
        y_val = float(current_xyz[1])
        z_val = float(current_xyz[2])

        # Buffer points for final 3D visualization
        print_path_points.append([x_val, y_val, z_val])
        
        # Simulate flow-rate control based on trajectory curvature
        pump_speed = 125.0 + 25.0 * math.sin(ang)

        # Sync data with TwinCAT PLC for real-time monitoring
        if plc_connected:
            try:
                plc.write_by_name('GVL_Printer.fCurrentX', x_val, pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fCurrentY', y_val, pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fCurrentZ', z_val, pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fPumpSpeed', pump_speed, pyads.PLCTYPE_REAL)
                print(f"[SYNC] TCP-> X:{x_val:.1f} Y:{y_val:.1f} Z:{z_val:.1f} | Pump RPM-> {pump_speed:.1f}")
            except:
                pass
        else:
            print(f"[OFFLINE] TCP-> X:{x_val:.1f} Y:{y_val:.1f} Z:{z_val:.1f} | Target RPM-> {pump_speed:.1f}")

        time.sleep(0.05)

except Exception as e:
    print(f"\n[FATAL] Execution failed: {repr(e)}")
finally:  
    # ---------------- 4. Graceful Shutdown (SAFETY FIRST) ----------------
    if plc_connected:
        try:
            # Safely halt pump and return state machine to 'Idle' (State 0)
            plc.write_by_name('GVL_Printer.fPumpSpeed', 0.0, pyads.PLCTYPE_REAL)
            plc.write_by_name('GVL_Printer.nPrinterState', 0, pyads.PLCTYPE_INT)
            print("\n[INFO] Job finished. Holding connection for 1.5s to verify final Idle state...")
            time.sleep(1.5)
            plc.close()
        except:
            pass
    print("\n[CLOSED] ADS connection safely released.")

    # ---------------- 5. 3D Trajectory Analysis Plotting ----------------
    print("\n[ACTION] Generating 3D Trajectory Plot for analysis...")
    if len(print_path_points) > 1:
        # Extract coordinates for Matplotlib
        xs = [pt[0] for pt in print_path_points]
        ys = [pt[1] for pt in print_path_points]
        zs = [pt[2] for pt in print_path_points]

        # Setup 3D plot
        fig = plt.figure("3D Concrete Printing Trajectory Analysis")
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot the extrusion path with high visibility
        ax.plot(xs, ys, zs, label='Extrusion Path', color='darkorange', linewidth=3)
        
        # Configure axis labels and aesthetics
        ax.set_title('Digital Twin Trajectory Verification', fontsize=14, fontweight='bold')
        ax.set_xlabel('X Axis (mm)')
        ax.set_ylabel('Y Axis (mm)')
        ax.set_zlabel('Z Axis (mm)')
        ax.legend()
        
        # Equalize aspect ratio to maintain dimensional accuracy
        ax.set_box_aspect([1, 1, 1])

        print("[SUCCESS] Rendering 3D interactive plot window. Close the window to terminate.")
        plt.show()