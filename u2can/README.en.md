# Damiao Motor Linux C++ Driver Library

This driver library is a modified version of the driver provided to Damiao customers, with many additional features. Minimum supported C++ standard is C++11.

This driver currently only works on **Linux**, and is especially suitable for users who use ROS.

**Join the QQ group 677900232 for Damiao motor technical discussions. Visit the Damiao store to browse products:** [Damiao Store - Taobao](https://shop290016675.taobao.com/?spm=pc_detail.29232929/evo365560b447259.shop_block.dshopinfo.59f47dd6w4Z4dX)

### 1. Include the Damiao library

Include the Damiao header:

```c++
#include "damiao.h"
```

### 2. Define control objects

To control a motor you need to define several objects.

First, create a serial port object. Damiao's USB-to-serial uses 921600 baud by default; choose the device path according to your Linux system.

```c++
auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
```

Then define the motor control object and motor objects. In the motor constructor, the first parameter is the motor type, the second is the motor's SlaveID (CAN ID), and the third is the MasterID (host ID). Do not set MasterID to 0x00. It is recommended to set MasterID as SlaveID + 0x10, for example 0x11 below.

**MasterID and SlaveID must be configured in the Damiao PC tool!! If problems occur, first check that MasterID does not conflict with SlaveID and is not 0x00.**

**Do not set MasterID to 0x00.**

```c++
auto dm = damiao::Motor_Control(serial);
damiao::Motor M1(damiao::DM4310, 0x01, 0x11);
damiao::Motor M2(damiao::DM4310, 0x05, 0x15);
```

### 3. Motor state

#### 3.1 Add motors

Add motors using addMotor:

```c++
dm.addMotor(&M1);
dm.addMotor(&M2);
```

#### 3.2 Enable motors

Recommendation: if you will modify motor parameters, enable the motors at the end.

```c++
dm.enable(M1);
dm.enable(M2);
```

For compatibility with older firmware, enabling might require specifying the mode (the motor must be enabled in the mode it currently uses; you cannot change the internal mode by calling enable_old).

```c++
dm.enable_old(Motor1, damiao::MIT_MODE);
dm.enable_old(Motor2, damiao::POS_VEL_MODE);
dm.enable_old(Motor3, damiao::VEL_MODE);
```

#### 3.3 Set motor zero position

With the motor disabled, move it to the desired zero position and run the following; the current position will be stored as the motor's zero.

```c++
dm.set_zero_position(M1);
dm.set_zero_position(M2);
```

#### 3.4 Disable motors

```c++
dm.disable(M1);
dm.disable(M2);
```

#### 3.5 Retrieve motor state

Damiao motors by default return torque, position, velocity, etc. only when a control frame is sent. If you want to get the motor state without sending a control command, use:

```c++
dm.refresh_motor_status(M1);
dm.refresh_motor_status(M2);
std::cout << "motor1--- POS:" << M1.Get_Position() << " VEL:" << M1.Get_Velocity() << " CUR:" << M1.Get_tau() << std::endl;
std::cout << "motor2--- POS:" << M2.Get_Position() << " VEL:" << M2.Get_Velocity() << " CUR:" << M2.Get_tau() << std::endl;
```

Use refresh_motor_status to obtain the current motor state and store it in the motor object.

### 4. Motor control modes

Recommendation: after each control command, delay 1–2ms. USB-to-CAN has internal buffering and may work without delay, but adding a delay is recommended.

#### 4.1 MIT mode

Default control is MIT mode. Parameters: motor object, kp, kd, position, velocity, torque. Refer to the Damiao manual for details; the library contains parameter documentation.

```c++
dm.control_mit(M1, 50, 0.3, 0, 0, 0);
```

#### 4.2 Position + Velocity mode

Parameters: motor object, target position, velocity to reach that position. See function docs for details.

```c++
float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
dm.control_pos_vel(M2, q * 10, 5);
```

#### 4.3 Velocity mode

Second parameter is the desired motor speed.

```c++
dm.control_vel(M1, q * 10);
```

#### 4.4 Position-Force hybrid mode

Parameters: motor object, position, velocity range (0–10000), current range (0–10000). See Damiao documentation for details.

```c++
dm.control_pos_force(M1, 5, 500, 1000);
```

### 5. Reading motor state

Motor state values are stored in the motor object. Note: Damiao motors update their state only after a control frame is sent or after calling refresh_motor_status.

Damiao uses a request-response model: the motor returns its state only after receiving a command, at which point the motor object is updated.

```c++
float pos = M1.Get_Position();  // get motor position
float vel = M1.Get_Velocity();  // get motor velocity
float tau = M1.Get_tau();       // get motor torque
```

### 6. Changing internal motor parameters

Newer Damiao firmware supports changing motor modes and other parameters via CAN. Requires firmware version 5013 or above. Consult Damiao support for details. Note: perform parameter changes while the motor is disabled.

#### 6.1 Change control mode

You can change the motor control mode online. Supported modes: MIT, POS_VEL, VEL, Torque_Pos. The functions return a boolean; true indicates success (but a false does not necessarily mean failure).

```c++
if (dm.switchControlMode(M1, damiao::MIT_MODE))
   std::cout << "Switch to MIT Success" << std::endl;
if (dm.switchControlMode(M2, damiao::POS_VEL_MODE))
   std::cout << "Switch to POS_VEL_MODE Success" << std::endl;
```

To persist the control mode, save parameters afterward.

#### 6.2 Save parameters

By default parameter changes are not saved to flash. Use the following to save changes to the motor's flash. Note: call this while the motor is disabled. The function internally disables the motor if needed and waits for parameter write completion.

```c++
dm.save_motor_param(M1);
dm.save_motor_param(M2);
```

#### 6.3 Read internal registers

Many internal parameters can be read via CAN. See the Damiao manual for parameter list. Readable parameters are enumerated in DM_Reg. Use read_motor_param to read values.

```c++
std::cout << "motor1 PMAX:" << dm.read_motor_param(M1, damiao::PMAX) << std::endl;
std::cout << "motor1 UV_Value:" << dm.read_motor_param(M1, damiao::UV_Value) << std::endl;
std::cout << "motor2 PMAX:" << dm.read_motor_param(M2, damiao::PMAX) << std::endl;
std::cout << "motor2 UV_Value:" << dm.read_motor_param(M2, damiao::UV_Value) << std::endl;
```

#### 6.4 Modify internal registers

Some internal registers are writable; others are read-only. Use change_motor_param to modify writable registers (for example, damiao::UV_Value). Changes will update the motor object's internal values.

Note: these changes are not persistent across power cycles unless you save parameters.

```c++
if (dm.change_motor_param(M1, damiao::UV_Value, 12.6f))
    std::cout << "Change UV_Value Success" << std::endl;
std::cout << "motor1 UV_Value:" << dm.read_motor_param(M1, damiao::UV_Value) << std::endl;

if (dm.change_motor_param(M2, damiao::UV_Value, 12.6f))
    std::cout << "Change UV_Value Success" << std::endl;
std::cout << "motor2 UV_Value:" << dm.read_motor_param(M2, damiao::UV_Value) << std::endl;

if (dm.change_motor_param(M1, damiao::CTRL_MODE, 1))
    std::cout << "Change CTRL_MODE Success" << std::endl;
std::cout << "motor1 CTRL_MODE:" << dm.read_motor_param(M1, damiao::CTRL_MODE) <<