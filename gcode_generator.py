# gcode_generator.py (仅用于一次性生成测试文件)
import math
import os

def generate_square_wall():
    with open("01_Square_Wall.gcode", "w") as f:
        extrusion = 0
        for layer in range(5):
            z = layer * 3.0
            pts = [(0,0), (150,0), (150,150), (0,150), (0,0)]
            for x, y in pts:
                extrusion += 1.0
                f.write(f"G1 X{x:.1f} Y{y:.1f} Z{z:.1f} E{extrusion:.1f}\n")
    print("✅ Created: 01_Square_Wall.gcode")

def generate_circle_column():
    with open("02_Circle_Column.gcode", "w") as f:
        extrusion = 0
        for layer in range(10):
            z = layer * 3.0
            for angle in range(0, 365, 10):
                rad = math.radians(angle)
                x = 100 + 100 * math.cos(rad)
                y = 100 + 100 * math.sin(rad)
                extrusion += 0.5
                f.write(f"G1 X{x:.1f} Y{y:.1f} Z{z:.1f} E{extrusion:.1f}\n")
    print("✅ Created: 02_Circle_Column.gcode")

def generate_vertico_pavilion():
    with open("03_Vertico_Pavilion.gcode", "w") as f:
        extrusion = 0
        for layer in range(15):
            z = layer * 3.0
            for angle in range(0, 365, 5):
                rad = math.radians(angle)
                r = 200 + 35 * math.sin(6 * rad)
                x = 300 + r * math.cos(rad)
                y = 0 + r * math.sin(rad)
                extrusion += 0.5
                f.write(f"G1 X{x:.1f} Y{y:.1f} Z{z:.1f} E{extrusion:.1f}\n")
    print("✅ Created: 03_Vertico_Pavilion.gcode")

generate_square_wall()
generate_circle_column()
generate_vertico_pavilion()
print("🎉 All test files generated successfully!")