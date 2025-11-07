# Motor Control Routine

A multi-language collection of control examples for DaMiao motors, including:

- C++ (Linux, suitable for ROS scenarios): `C++example/u2can`
- Python (Windows/macOS/Linux): `Pythonexample/u2can`
- STM32 bare-metal examples (F4/H7, CAN/FDCAN): `stm32example/`

Accompanying document: `DM-J4310-2EC_V1.1_Gear_Motor_User_Manual_English.pdf`


## Directory Structure

- `C++example/u2can/`: C++ driver and examples on Linux, based on serial-to-CAN.
- `Pythonexample/u2can/`: Python driver and examples.
- `stm32example/`: STM32 bare-metal projects (F4/H7), including CAN/FDCAN, PID, driver, and control module source code.

