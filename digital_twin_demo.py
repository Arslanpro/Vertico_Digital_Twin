import pyads
from robodk.robolink import *
from robodk.robomath import *
import time
import math

# ---------------- 1. TwinCAT PLC Connection Setup ----------------
plc_connected = False
AMS_NET_ID = '192.168.0.109.1.1'
plc = pyads.Connection(AMS_NET_ID, pyads.PORT_TC3PLC1)

# ---------------- 2. RoboDK Simulation Setup ----------------
RDK = Robolink()
robot = RDK.Item('', ITEM_TYPE_ROBOT) 

if not robot.Valid():
    print("[ERROR] Robot not found in RoboDK. Please load the station first.")
    exit()

# Get the robot's base frame to ensure generated 3D trajectories align perfectly
reference_frame = robot.Parent()

# Try establishing ADS connection to the soft-PLC
try:
    plc.open()
    plc.write_by_name('GVL_Printer.bSystemReady', True, pyads.PLCTYPE_BOOL)
    plc_connected = True
    print("[SUCCESS] Connected to TwinCAT PLC Real-time Kernel.\n")
except Exception as e:
    print("[WARNING] TwinCAT Kernel access blocked (likely due to IT VBS policy).")
    print("[INFO] Fallback activated: Switching to 'Offline 3D Simulation' mode...\n")
    plc_connected = False

try:
    # --- Kinematic Singularity Avoidance ---
    print("[ACTION] Moving robot to a safe Home Pose to avoid singularities...")
    
    # Hardcoded safe joint values: [Base, Shoulder, Elbow, Wrist1, Wrist2(pointing down), Wrist3]
    safe_joints = [0, 30, 30, 0, 90, 0]
    
    # Use MoveJ (Joint Interpolation) for safe, unrestricted space movement
    robot.MoveJ(safe_joints) 
    time.sleep(1.5) 

    # Record this safe pose as the starting reference frame for the print job
    pose_ref = robot.Pose()
    print("[READY] Pose calibration complete. Executing 3D Concrete Printing G-code...")

    heartbeat_status = False

    # --- Handshake Initialization ---
    if plc_connected:
        # Toggle heartbeat to clear any existing Watchdog errors in the PLC
        heartbeat_status = not heartbeat_status
        plc.write_by_name('GVL_Printer.bPythonHeartbeat', heartbeat_status, pyads.PLCTYPE_BOOL)
        time.sleep(0.1) 
        # Shift PLC state machine to 'Printing' (State 1)
        plc.write_by_name('GVL_Printer.nPrinterState', 1, pyads.PLCTYPE_INT)

    # Create a folder in RoboDK to keep the generated trajectory lines organized
    print_group = RDK.AddFolder('Concrete_Print_Result')
    prev_xyz = None # Buffer to store the previous point for line drawing

    # ---------------- 3. Main Digital Twin Loop ----------------
    for i in range(100):

        # Update Watchdog toggle bit to prove Python orchestrator is alive
        if plc_connected:
            heartbeat_status = not heartbeat_status
            plc.write_by_name('GVL_Printer.bPythonHeartbeat', heartbeat_status, pyads.PLCTYPE_BOOL)

        ang = math.radians(i * 10)
        
        # Calculate Spiral Trajectory
        # Subtracting 150 from X ensures the path starts exactly at the current TCP position (0 displacement at ang=0)
        dx = 150 * math.cos(ang) - 150  
        dy = 150 * math.sin(ang)
        dz = i * 1.5 # Simulate layer height increment
        
        target_pose = transl(dx, dy, dz) * pose_ref
        
        # Use MoveL (Linear Interpolation) to strictly follow a straight Cartesian path (Mandatory for 3D printing)
        robot.MoveL(target_pose)
        
        # Extract absolute XYZ coordinates
        current_xyz = target_pose.Pos()
        x_val, y_val, z_val = current_xyz[0], current_xyz[1], current_xyz[2]

        # --- Phase 1: Trajectory Visualization ---
        if prev_xyz is not None:
            # Draw a physical line segment in RoboDK between the last point and current point
            segment = RDK.AddCurve([prev_xyz, current_xyz], reference_frame)
            segment.setParent(print_group) 
            
            # Set material color to Grey (simulating concrete) -> [R, G, B, Alpha]
            segment.setColor([0.5, 0.5, 0.5, 1.0])

        prev_xyz = current_xyz
        
        # Calculate dynamic pump speed based on trajectory curve (simulating flow-rate control)
        pump_speed = 125.0 + 25.0 * math.sin(ang)

        # Write parameters to TwinCAT Kernel
        if plc_connected:
            try:
                plc.write_by_name('GVL_Printer.fCurrentX', float(x_val), pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fCurrentY', float(y_val), pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fCurrentZ', float(z_val), pyads.PLCTYPE_REAL)
                plc.write_by_name('GVL_Printer.fPumpSpeed', float(pump_speed), pyads.PLCTYPE_REAL)
                print(f"[SYNC] TCP-> X:{x_val:.1f} Y:{y_val:.1f} Z:{z_val:.1f} | Pump RPM-> {pump_speed:.1f}")
            except:
                pass
        else:
            print(f"[OFFLINE] TCP-> X:{x_val:.1f} Y:{y_val:.1f} Z:{z_val:.1f} | Target RPM-> {pump_speed:.1f}")

        # Sleep to simulate actual print speed and allow visual observation
        time.sleep(0.05)

except Exception as e:
    print(f"\n[FATAL] Execution failed: {repr(e)}")
finally:  
    # --- Graceful Shutdown ---
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
    print("\n[CLOSED] ADS connection safely released. Demo terminated.")