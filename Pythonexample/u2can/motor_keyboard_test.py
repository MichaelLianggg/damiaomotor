#!/usr/bin/env python3
"""
Keyboard-driven motor test script (Python) - MIT Mode

Controls:
  w/W : increase target position
  s/S : decrease target position
  a/A : set target to min
  d/D : set target to max
  r/R : reset target to 0
  j/J : increase Kp (proportional gain)
  k/K : decrease Kp (proportional gain)
  i/I : increase Kd (derivative gain)
  l/L : decrease Kd (derivative gain)
  q/Q : enable motor
  e/E : disable motor
  p/P : print detailed status
  ESC : exit

This script uses MIT control mode with the Motor and MotorControl classes from DM_CAN.py.
"""

import sys
import time
import termios
import tty
import select
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable
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
    print("Keyboard controls (MIT Mode):")
    print("  w/s: increase/decrease target position")
    print("  a/d: set target to min/max")
    print("  r: reset target to 0")
    print("  j/k: increase/decrease Kp (proportional gain)")
    print("  i/l: increase/decrease Kd (derivative gain)")
    print("  q: enable motor, e: disable motor")
    print("  p: print detailed status, ESC: exit")


def main():
    # Parameters for MIT mode
    position_step = 0.5
    max_position = 12.5
    min_position = -12.5
    kp = 10.0  # MIT mode proportional gain
    kd = 1.0   # MIT mode derivative gain
    kp_step = 1.0
    kd_step = 0.1
    target_torque = 0.0  # Feedforward torque
    torque_step = 0.5

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
    m1 = Motor(DM_Motor_Type.DM4310, 0x002, 0x052)
    mc.addMotor(m1)

    print("Initializing motor...")
    mc.disable(m1)
    time.sleep(1)

    # Read some parameters to establish communication with motor
    print("Reading motor parameters...")
    pmax = mc.read_motor_param(m1, DM_variable.PMAX)
    vmax = mc.read_motor_param(m1, DM_variable.VMAX)
    tmax = mc.read_motor_param(m1, DM_variable.TMAX)
    print(f"Motor params - PMAX: {pmax}, VMAX: {vmax}, TMAX: {tmax}")
    time.sleep(0.5)

    # Switch to MIT mode
    if mc.switchControlMode(m1, Control_Type.MIT):
        print("Switched to MIT mode")
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
                        kp = min(kp + kp_step, 500)
                        print(f"\n[INFO] Kp increased to: {kp}")
                    elif key == 'k':
                        kp = max(kp - kp_step, 0)
                        print(f"\n[INFO] Kp decreased to: {kp}")
                    elif key == 'i':
                        kd = min(kd + kd_step, 5)
                        print(f"\n[INFO] Kd increased to: {kd}")
                    elif key == 'l':
                        kd = max(kd - kd_step, 0)
                        print(f"\n[INFO] Kd decreased to: {kd}")
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
                        print("\n=== Motor Status (MIT Mode) ===")
                        print(f"Current Position: {current_position} rad")
                        print(f"Target Position: {target_position} rad")
                        print(f"Actual Torque: {m1.getTorque()} Nm")
                        print(f"Kp: {kp}")
                        print(f"Kd: {kd}")
                        print(f"Target Torque: {target_torque} Nm")
                        print(f"Status: {'ENABLED' if motor_enabled else 'DISABLED'}")
                        print("=========================================")

                # control and refresh when motor enabled
                if motor_enabled:
                    current_position = m1.getPosition()
                    # MIT control: kp, kd, target_position, target_velocity, target_torque
                    mc.controlMIT(m1, kp, kd, target_position, 0, target_torque)

                mc.refresh_motor_status(m1)

                # Print a compact status on one line (overwrite)
                status_line = (f"\r[POS: {current_position:7.2f} rad | TGT: {target_position:7.2f} rad | "
                               f"Kp: {kp:5.1f} | Kd: {kd:4.2f} | "
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
