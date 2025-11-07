# DaMiao Motor Linux C++ Driver Library
This driver library is a modified version of the driver provided to DaMiao customers, with many additional features. Minimum supported C++ version is C++11.

This driver library currently only works on **Linux**, and is especially suitable for ROS users.

**Welcome to join QQ group: 677900232 for DaMiao motor technical discussions. Visit the DaMiao store for product purchases** [DaMiao Intelligent Control Store - Taobao](https://shop290016675.taobao.com/?spm=pc_detail.29232929/evo365560b447259.shop_block.dshopinfo.59f47dd6w4Z4dX)

### 1. Include DaMiao Library

​	Include the DaMiao library through the header file

```c++
#include "damiao.h"
```

### 2. Define Control Objects

​	To control the motor, you need to define related objects.

​	First is the serial port object. DaMiao's USB-to-serial port has a default baud rate of 921600. Select the serial port number according to your Linux device's port designation.

```c++
auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
```

​	Then you need to define the motor control object and motor objects. In the motor object, the first parameter is the motor type which needs to be selected, the second parameter is the motor's SlaveID (CAN ID), which is the motor control ID, **the third parameter is MasterID (host ID). The MasterID should not be set to 0x00. It is recommended to set it to CANID + 0x10, for example 0x11 as shown below.**

**MasterID and SlaveID need to be set in DaMiao's upper computer software! If problems occur, please first check that MasterID does not conflict with SlaveID and is not 0x00**

**DO NOT set MasterID to 0x00**

**DO NOT set MasterID to 0x00**

**DO NOT set MasterID to 0x00**

```c++
auto dm =damiao::Motor_Control(serial);
damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4310,0x05, 0x15);
```

### 3. Motor Status

#### 3.1 Add Motors

Add motors using addMotor.

```c++
dm.addMotor(&M1);
dm.addMotor(&M2);
```

#### 3.2 Enable Motors

**Recommendation: If you want to modify motor parameters, it's recommended to enable the motors last**

```c++
dm.enable(M1);
dm.enable(M2);
```

​	This code is for compatibility with older firmware. For old motor firmware versions, enabling requires specifying the corresponding mode (i.e., you need to enable the motor with the corresponding mode, but cannot modify the motor's current mode). **Note: you need to enable the motor with its current corresponding mode, and cannot modify the internal mode of the motor**

```c++
dm.enable_old(Motor1,damiao::MIT_MODE);
dm.enable_old(Motor2,damiao::POS_VEL_MODE);
dm.enable_old(Motor3,damiao::VEL_MODE);
```

#### 3.3 Set Motor Zero Position

Place the motor in the desired zero position while disabled, then run the following two lines. The motor will set the current position as its zero point.

```c++
dm.set_zero_position(M1);
dm.set_zero_position(M2);
```

#### 3.4 Disable Motors

```c++
dm.disable(M1);
dm.disable(M2);
```

#### 3.5 Get Motor Status

By default, DaMiao motors require sending a control frame to receive current motor torque, position, velocity, and other information. If you want to get the motor status without sending a control command, you can use the following command.

```c++
dm.refresh_motor_status(M1);
dm.refresh_motor_status(M2);
std::cout<<"motor1--- POS:"<<M1.Get_Position()<<" VEL:"<<M1.Get_Velocity()<<" CUR:"<<M1.Get_tau()<<std::endl;
std::cout<<"motor2--- POS:"<<M2.Get_Position()<<" VEL:"<<M2.Get_Velocity()<<" CUR:"<<M2.Get_tau()<<std::endl;
```

Using the **refresh_motor_status** function, you can get the current motor status and save it to the corresponding motor object.

### 4. Motor Control Modes

**It is recommended to add a delay of 2ms or 1ms after each control frame. USB-to-CAN converters usually have a buffer, so they can work without delays, but adding a delay is recommended.**

#### 4.1 MIT Mode

​	The default control is MIT control mode. The first parameter is the motor object, second is kp, third is kd, fourth is position, fifth is velocity, and sixth is torque. Please refer to the DaMiao manual for control details. Parameter descriptions are detailed in the library.

```c++
dm.control_mit(M1, 50, 0.3, 0, 0, 0);
```

#### 4.2 Position-Velocity Mode

​	The first parameter is the motor object, the second parameter is the target position, and the third parameter is the velocity used to rotate to that position. Detailed parameter descriptions are included in the function documentation.

```c++
float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
dm.control_pos_vel(M2, q*10, 5);
```

#### 4.3 Velocity Mode

​	The second parameter is the motor's rotational speed.

```c++
dm.control_vel(M1, q*10);
```

#### 4.4 Position-Force Hybrid Mode

​	The first is the motor object, the second is motor position, the third is motor velocity ranging from 0-10000, and the fourth is motor current ranging from 0-10000. For detailed information, please refer to the DaMiao documentation.

```c++
dm.control_pos_force(M1,5,500, 1000);
```

### 5. Read Motor Status

All motor states are saved in the corresponding motor object and can be accessed using the following functions.

**Please note! DaMiao motor status is only refreshed after sending a control frame or refreshing the status (using the refresh_motor_status function)!**

**DaMiao motors operate in send-and-receive mode. The motor will only return its current status when a command is sent, and only then will the motor update**

```c++
float pos = M1.Get_Position();  //Get motor position
float vel = M1.Get_Velocity();  //Get motor velocity
float tau = M1.Get_tau();       //Get motor torque
```

### 6. Modify Motor Internal Parameters

DaMiao motors with new firmware support modifying motor modes and other parameters via CAN. This requires firmware version 5013 or higher. Please consult DaMiao customer service for details. **Please note: all parameter saving and modifications must be done in disabled mode!**

#### 6.1 Change Motor Control Mode

The following function can be used to modify the motor's control mode. Supports online modification of four control modes: MIT, POS_VEL, VEL, and Torque_Pos. Below is a demo. The code will return a value; if it's True, the setting was successful. If not, it doesn't necessarily mean it failed, haha. **Please note that mode modification is only effective for the current session. After power-off, the mode will revert to the previous setting**

```c++
if(dm.switchControlMode(M1, damiao::MIT_MODE))
   std::cout << "Switch to MIT Success" << std::endl;
if(dm.switchControlMode(M2, damiao::POS_VEL_MODE))
   std::cout << "Switch to POS_VEL_MODE Success" << std::endl;
```

**To maintain the motor control mode, you need to save the parameters at the end**

#### 6.2 Save Parameters

​	By default, after modifying motor modes and other operations, parameters will not be saved to flash. You need to use the following command to save to the motor's flash. Here's an example. **Please note that this code saves all modifications to Motor1's flash, and must be done in disabled mode**. The function includes automatic disabling code to prevent parameters from being unsavable when the motor is enabled. The function includes a delay to wait for parameter modification to complete.

```c++
dm.save_motor_param(M1);
dm.save_motor_param(M2);
```

#### 6.3 Read Internal Register Parameters

​	Many internal register parameters can be read via CAN. Please refer to the DaMiao manual for the complete parameter list. All readable parameters are already in the DM_Reg enumeration class. You can read them using read_motor_param.

```c++
std::cout<<"motor1 PMAX:"<<dm.read_motor_param(M1, damiao::PMAX)<<std::endl;
std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M1, damiao::UV_Value)<<std::endl;
std::cout<<"motor2 PMAX:"<<dm.read_motor_param(M2, damiao::PMAX)<<std::endl;
std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M2, damiao::UV_Value)<<std::endl;
```

#### 6.4 Modify Internal Register Parameters

​	Some internal registers support modification, while others are read-only (cannot be modified). By calling the change_motor_param function, you can modify internal register values (such as damiao::UV_Value). Like reading registers above, the register values will also be synchronized to the internal values of the motor object.

**Please note that modifying internal register parameters will revert after power-off and are not saved**

```c++
if(dm.change_motor_param(M1, damiao::UV_Value, 12.6f))
    std::cout << "Change UV_Value Success" << std::endl;
std::cout<<"motor1 UV_Value:"<<dm.read_motor_param(M1, damiao::UV_Value)<<std::endl;
if(dm.change_motor_param(M2, damiao::UV_Value, 12.6f))
    std::cout << "Change UV_Value Success" << std::endl;
std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M2, damiao::UV_Value)<<std::endl;
if(dm.change_motor_param(M1,damiao::CTRL_MODE,1))
    std::cout << "Change CTRL_MODE Success" << std::endl;
std::cout<<"motor1 CTRL_MODE:"<<dm.read_motor_param(M1, damiao::CTRL_MODE)<<std::endl;
```

