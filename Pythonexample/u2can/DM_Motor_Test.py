import math
from DM_CAN import *
import serial
from serial.tools import list_ports
import time
import sys

# Auto-detect serial port device
# def find_serial_port():
#     """Automatically find available serial port, prefer CDC Device (VID:PID=2E88:4603) or first available port"""
#     ports = list_ports.comports()
#     if not ports:
#         print("Error: No serial ports found, please check device connection.")
#         sys.exit(1)
    
#     # Prioritize finding device with specific VID:PID (your u2can device)
#     for port in ports:
#         if port.vid == 0x2E88 and port.pid == 0x4603:
#             print(f"Found u2can device: {port.device} ({port.description})")
#             return port.device
    
#     # If specific device not found, use first available port
#     print(f"Using first available serial port: {ports[0].device} ({ports[0].description})")
#     return ports[0].device

Motor1=Motor(DM_Motor_Type.DM4310,0x03, 0x53)
Motor2=Motor(DM_Motor_Type.DM4310,0x02,0x52)
#serial_device = serial.Serial(find_serial_port(), 921600, timeout=0.5)
serial_device = serial.Serial("/dev/ttyACM0", 921600, timeout=1)
MotorControl1=MotorControl(serial_device)
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)

if MotorControl1.switchControlMode(Motor2,Control_Type.Torque_Pos):
    print("switch POS_VEL success")
# c
print("sub_ver:",MotorControl1.read_motor_param(Motor1,DM_variable.sub_ver))
print("Gr:",MotorControl1.read_motor_param(Motor1,DM_variable.Gr))

# if MotorControl1.change_motor_param(Motor1,DM_variable.KP_APR,54):
#     print("write success")
print("PMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor1,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.TMAX))
print("Motor2:")
print("PMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor2,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.TMAX))
# MotorControl1.enable(Motor3)
MotorControl1.save_motor_param(Motor1)
MotorControl1.save_motor_param(Motor2)
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
i=0
while i<10000:
    q=math.sin(time.time())
    i=i+1
    MotorControl1.control_pos_force(Motor2, 2000, 1000,100)
    # MotorControl1.control_Vel(Motor1, q*5)
   # MotorControl1.control_Pos_Vel(Motor1,q*8,30)
    # print("Motor1:","POS:",Motor1.getPosition(),"VEL:",Motor1.getVelocity(),"TORQUE:",Motor1.getTorque())
    # MotorControl1.controlMIT(Motor2, 35, 0.1, 8*q, 0, 0)

    #MotorControl1.control_Vel(Motor2, 8*q)
    # print("Motor2:","POS:",Motor2.getPosition(),"VEL:",Motor2.getVelocity(),"TORQUE:",Motor2.getTorque())
    #print(Motor2.getTorque())
    # print(Motor2.getTorque())
    time.sleep(0.001)
    # MotorControl1.control(Motor3, 50, 0.3, q, 0, 0)

# Close serial port when finished
serial_device.close()