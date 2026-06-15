import pyads
from robodk.robolink import *
from robodk.robomath import *
import time
import math

# ---------------- 1. TwinCAT PLC 连接设置 ----------------
AMS_NET_ID = '192.168.0.109.1.1'
plc = pyads.Connection(AMS_NET_ID, pyads.PORT_TC3PLC1)

# ---------------- 2. RoboDK 仿真软件连接设置 ----------------
RDK = Robolink()
robot = RDK.Item('', ITEM_TYPE_ROBOT) 

if not robot.Valid():
    print("❌ 未在 RoboDK 中找到机械臂！请确保 RoboDK 软件已打开且模型已加载。")
    exit()

try:
    # 开启 PLC 连接
    plc.open()
    print("🔌 成功连接到底层 TwinCAT PLC！")
    plc.write_by_name('GVL_Printer.bSystemReady', True, pyads.PLCTYPE_BOOL)
    print("✅ 系统已就绪！\n")

    # 【核心修复：解开奇点 (Singularity) 与安全启动】
    print("🔧 正在将机械臂移动到安全的初始工作姿态 (避开数学奇点)...")
    # 设定 6 个关节的安全角度：[底座, 大臂前倾, 小臂下压, 旋转0, 手腕下折(喷头朝下), 末端0]
    safe_joints = [0, 30, 30, 0, 90, 0]
    
    # 使用纯关节运动 (MoveJ) 强制走到安全位置，这在底层不会触发奇点报错
    robot.MoveJ(safe_joints) 
    time.sleep(1.5) # 稍微停顿一下，让你看清机械臂“低头准备”的帅气动作

    # 获取低头后的这个安全姿态作为后续计算的基准
    pose_ref = robot.Pose()
    print("🚀 姿态标定完成，开始执行螺旋形 3D 打印 G-code...")

    heartbeat_status = False

    plc.write_by_name('GVL_Printer.nPrinterState', 1, pyads.PLCTYPE_INT)

    # ---------------- 3. 数字孪生主循环 ----------------
    for i in range(1000):

        heartbeat_status = not heartbeat_status
        plc.write_by_name('GVL_Printer.bPythonHeartbeat', heartbeat_status, pyads.PLCTYPE_BOOL)

        ang = math.radians(i * 10)
        
        # 螺旋轨迹规划 (半径 150mm)
        dx = 150 * math.cos(ang) - 150  
        dy = 150 * math.sin(ang)
        dz = i * 1.5 
        
        # 计算目标姿态
        target_pose = transl(dx, dy, dz) * pose_ref
        
        # 1. 驱动 RoboDK 物理机械臂移动 (直线插补)
        robot.MoveL(target_pose)
        
        # 2. 提取 XYZ 绝对坐标
        current_xyz = target_pose.Pos()
        x_val, y_val, z_val = current_xyz[0], current_xyz[1], current_xyz[2]
        
        # 3. 计算随动泵速
        pump_speed = 125.0 + 25.0 * math.sin(ang)

        # 4. 写入 TwinCAT 底层内核
        plc.write_by_name('GVL_Printer.fCurrentX', float(x_val), pyads.PLCTYPE_REAL)
        plc.write_by_name('GVL_Printer.fCurrentY', float(y_val), pyads.PLCTYPE_REAL)
        plc.write_by_name('GVL_Printer.fCurrentZ', float(z_val), pyads.PLCTYPE_REAL)
        plc.write_by_name('GVL_Printer.fPumpSpeed', float(pump_speed), pyads.PLCTYPE_REAL)

        # 终端打印反馈
        print(f"🔄 [打印中] 坐标-> X:{x_val:.1f} Y:{y_val:.1f} Z:{z_val:.1f} | 泵速-> {pump_speed:.1f}")

        time.sleep(0.1)

except Exception as e:
    print(f"\n❌ 运行发生错误: {repr(e)}")
finally:
    plc.write_by_name('GVL_Printer.fPumpSpeed', 0.0, pyads.PLCTYPE_REAL)
    plc.write_by_name('GVL_Printer.nPrinterState', 0, pyads.PLCTYPE_INT)
    plc.close()
    print("\n🔒 演示结束，底层连接已安全释放。")