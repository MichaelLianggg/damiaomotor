#!/usr/bin/env python3
"""
Keyboard-driven motor test script (Python)

Controls (same mapping as provided C++ example):
  w/W : increase target position
  s/S : decrease target position
  a/A : set target to min
  d/D : set target to max
  r/R : reset target to 0
  j/J : increase velocity parameter
  k/K : decrease velocity parameter
  i/I : increase torque limit
  l/L : decrease torque limit
  q/Q : enable motor
  e/E : disable motor
  p/P : print detailed status
  ESC : exit

This script reuses the Motor and MotorControl classes from DM_CAN.py.
"""

import sys
import time
import termios
import tty
import select
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type
import serial
from serial.tools import list_ports


# def find_serial_port():
#     """Auto-detect serial port, prefer u2can VID:PID then first available."""
#     ports = list_ports.comports()
#     if not ports:
#         raise RuntimeError("No serial ports found. Please connect the device.")
#     for port in ports:
#         if port.vid == 0x2E88 and port.pid == 0x4603:
#             print(f"Found u2can device: {port.device} ({port.description})")
#             return port.device
#     print(f"Using first available serial port: {ports[0].device} ({ports[0].description})")
#     return ports[0].device
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

class KeyReader:
    """Non-blocking single-key reader using termios and select."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def get_key(self, timeout=0.0):
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            return ch
        return None


def print_instructions():
    print("Keyboard controls:")
    print("  w/s: increase/decrease target position")
    print("  a/d: set target to min/max")
    print("  r: reset target to 0")
    print("  j/k: increase/decrease velocity parameter")
    print("  i/l: increase/decrease torque limit")
    print("  q: enable motor, e: disable motor")
    print("  p: print detailed status, ESC: exit")


def main():
    # Parameters (same defaults as C++ example)
    position_step = 0.5
    max_position = 12.5
    min_position = -12.5
    current_velocity = 500
    current_torque = 2000
    velocity_step_force = 100
    torque_step = 500

    motor_enabled = False
    current_position = 0.0
    target_position = 0.0

    try:
       # port = find_serial_port()
        #serial_dev = serial.Serial(port, 921600, timeout=0.5)
        serial_dev = serial.Serial("/dev/ttyACM0", 921600, timeout=1)
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    mc = MotorControl(serial_dev)
    m1 = Motor(DM_Motor_Type.DM4310, 0x02, 0x52)
    mc.addMotor(m1)

    print("Initializing motor...")
   # mc.disable(m1)
    time.sleep(1)

    # Switch to position-force (hybrid) mode, corresponds to Torque_Pos
    if mc.switchControlMode(m1, Control_Type.Torque_Pos):
        print("Switched to POS_FORCE/Torque_Pos mode")
    else:
        print("Warning: failed to switch control mode")

    mc.save_motor_param(m1)
    mc.enable(m1)
    motor_enabled = True
    time.sleep(1)

    #print_instructions()
    print("Starting control loop. Press ESC to exit.")

    try:
        with KeyReader() as kr:
            while True:
                ch = kr.get_key(0.05)
                if ch:
                    # handle special ESC
                    if ord(ch) == 27:
                        print("\nExit requested (ESC).")
                        break

                    key = ch.lower()
                    if key == 'w':
                        target_position = min(target_position + position_step, max_position)
                        print(f"\n[INFO] Target position increased to: {target_position} rad")
                    elif key == 's':
                        target_position = max(target_position - position_step, min_position)
                        print(f"\n[INFO] Target position decreased to: {target_position} rad")
                    elif key == 'a':
                        target_position = min_position
                        print(f"\n[INFO] Target position set to minimum: {target_position} rad")
                    elif key == 'd':
                        target_position = max_position
                        print(f"\n[INFO] Target position set to maximum: {target_position} rad")
                    elif key == 'r':
                        target_position = 0.0
                        print(f"\n[INFO] Target position reset to 0 rad")
                    elif key == 'j':
                        current_velocity = min(current_velocity + velocity_step_force, 10000)
                        print(f"\n[INFO] Velocity parameter increased to: {current_velocity}")
                    elif key == 'k':
                        current_velocity = max(current_velocity - velocity_step_force, 0)
                        print(f"\n[INFO] Velocity parameter decreased to: {current_velocity}")
                    elif key == 'i':
                        current_torque = min(current_torque + torque_step, 10000)
                        print(f"\n[INFO] Torque limit increased to: {current_torque}")
                    elif key == 'l':
                        current_torque = max(current_torque - torque_step, 0)
                        print(f"\n[INFO] Torque limit decreased to: {current_torque}")
                    elif key == 'q':
                        if not motor_enabled:
                            mc.enable(m1)
                            motor_enabled = True
                            print("\n[INFO] Motor enabled")
                    elif key == 'e':
                        if motor_enabled:
                            mc.disable(m1)
                            motor_enabled = False
                            target_position = 0.0
                            print("\n[INFO] Motor disabled")
                    elif key == 'p':
                        print("\n=== Motor Status (Force-Position Mode) ===")
                        print(f"Current Position: {current_position} rad")
                        print(f"Target Position: {target_position} rad")
                        print(f"Actual Torque: {m1.getTorque()} Nm")
                        print(f"Velocity Parameter: {current_velocity}")
                        print(f"Torque Limit: {current_torque}")
                        print(f"Status: {'ENABLED' if motor_enabled else 'DISABLED'}")
                        print("=========================================")

                # control and refresh when motor enabled
                if motor_enabled:
                    current_position = m1.getPosition()
                    # pass velocity and torque parameters to control_pos_force
                    mc.control_pos_force(m1, target_position, int(current_velocity), int(current_torque))

                mc.refresh_motor_status(m1)

                # Print a compact status on one line (overwrite)
                status_line = (f"\r[POS: {current_position:7.2f} rad | TGT: {target_position:7.2f} rad | "
                               f"TAU: {current_torque:5d} | VEL: {current_velocity:5d} | "
                               f"ACT: {m1.getTorque():6.2f} Nm | "
                               f"Status: {'ENABLED' if motor_enabled else 'DISABLED'} ]   ")
                sys.stdout.write(status_line)
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        print("\nDisabling motor and closing serial port...")
        try:
            mc.disable(m1)
        except Exception:
            pass
        try:
            serial_dev.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
