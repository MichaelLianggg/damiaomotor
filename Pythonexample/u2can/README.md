# DM Motor Python Library

This library supports macOS, Linux, and Windows platforms.


### 1. Import DM Library

The default folder contains `DM_CAN.py` which is the motor library. To use it:

```python
from DM_CAN import *	
```

The motor library depends on `serial` and `numpy`. Remember to install these dependencies. **The specific requirements are in `requirements.txt` in the directory. These are the tested versions, but theoretically, lower versions should also work.**

### 2. Define Control Objects

Define motor objects - create as many as you have motors. Important: **Do NOT set MasterID to 0x00**

```python
Motor1=Motor(DM_Motor_Type.DM4310,0x01,0x11)
Motor2=Motor(DM_Motor_Type.DM4310,0x02,0x12)
Motor3=Motor(DM_Motor_Type.DM4310,0x03,0x13)
```

The first parameter is the motor type, the second is SlaveID (the motor's CAN ID), and the third parameter is MasterID (host ID). It's recommended that each MasterID is unique and higher than its corresponding SlaveID.

For example, Motor1 has SlaveID 0x01 and MasterID 0x11. This is the best practice.

**MasterID and SlaveID must be configured in the DaMiao GUI tool! If you encounter issues, first check that MasterID does not conflict with SlaveID and is NOT 0x00**

**Do NOT set MasterID to 0x00**

**Do NOT set MasterID to 0x00**

**Do NOT set MasterID to 0x00**

Python uses serial port with baudrate 921600. Select the appropriate port.

```python
serial_device = serial.Serial('COM0', 921600, timeout=0.5)
```

Initialize the motor control object. Pass the defined serial object as parameter:

```python
MotorControl1=MotorControl(serial_device)
```

### 3. Motor States

#### 3.1 Add Motors

Use `addMotor` to add motors:

```python
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
```

#### 3.2 Enable Motors

**Recommendation: If you need to modify motor parameters, enable motors last**

```python
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)
```

This code is for compatibility with old firmware. For older motor firmware versions, enabling requires specifying the mode (you must enable the motor's current mode, it does NOT change the motor's internal mode) **Note: You must enable the motor's current mode, this does NOT modify the motor's internal mode**

```python
MotorControl1.enable_old(Motor1,Control_Type.MIT)
MotorControl1.enable_old(Motor2,Control_Type.POS_VEL)
MotorControl1.enable_old(Motor3,Control_Type.VEL)
```

#### 3.3 Set Motor Zero Position

While the motor is disabled, move it to the position you want to set as zero, then run the following commands. The motor will set the current position as its zero point:

```python
MotorControl1.set_zero_position(Motor3)
MotorControl1.set_zero_position(Motor6)
```

#### 3.4 Disable Motors

```python
MotorControl1.disable(Motor3)
MotorControl1.disable(Motor6)
```

#### 3.5 Get Motor Status

DM motors by default only return current motor torque, position, and velocity information after each control command is sent. If you want to get the motor's current state without sending a control command, use the following:

```python
MotorControl1.refresh_motor_status(Motor1)
print("Motor1:","POS:",Motor1.getPosition(),"VEL:",Motor1.getVelocity(),"TORQUE:",Motor1.getTorque())
```

The `refresh_motor_status` function retrieves the current motor status and saves it to the corresponding motor object.

### 4. Motor Control Modes

**Recommended: Add a 1-2ms delay after each control frame. USB-to-CAN adapters have built-in buffers so no delay may work, but adding delay is recommended.**

#### 4.1 MIT Mode

After enabling the motor, you can use MIT mode control. MIT mode is recommended:

```python
MotorControl1.controlMIT(Motor1, 50, 0.3, 0, 0, 0)
```

#### 4.2 Position-Velocity Mode

Position-velocity mode: first parameter is motor object, second is position, third is rotation velocity. Detailed parameter descriptions are written in the function documentation, visible in IDEs like PyCharm.

Example:

```python
q=math.sin(time.time())
MotorControl1.control_Pos_Vel(Motor1,q*10,2)
```

#### 4.3 Velocity Mode

Example: first parameter is motor object, second is motor velocity

```python
q=math.sin(time.time())
MotorControl1.control_Vel(Motor1, q*5)
```

The new DM firmware supports mode switching.

#### 4.4 Position-Torque Hybrid Mode

First parameter is motor object, second is motor position, third is motor velocity (range 0-10000), fourth is motor current (range 0-10000). For details, see the DM documentation.

Example:

```python
MotorControl1.control_pos_force(Motor1, 10, 1000,100)
```

### 5. Reading Motor Status

All motor states are stored in the corresponding motor object. Use the following functions to retrieve them:

**Important! DM motor status is only updated after sending a control frame or calling the `refresh_motor_status` function!**

**DM motors use a send-receive pattern. The motor only returns current status after receiving a command, then the motor state updates**

```python
vel = Motor1.getVelocity()    # Get motor velocity
pos = Motor1.getPosition()    # Get motor position
tau = Motor1.getTorque()      # Get motor output torque
```

```python
MotorControl1.refresh_motor_status(Motor1)
print("Motor1:","POS:",Motor1.getPosition(),"VEL:",Motor1.getVelocity(),"TORQUE:",Motor1.getTorque())
```

### 6. Modifying Internal Motor Parameters

DM motors with new firmware support modifying motor modes and other parameters via CAN. Requires firmware version 5013 or higher. For details, contact DM customer service. **Important: All parameter saving and modifications must be done in DISABLED mode!**

#### 6.1 Change Motor Control Mode

Use the following function to modify the motor's control mode. Supports online modification of four control modes: MIT, POS_VEL, VEL, Torque_Pos. Below is a demo. The code returns a value - if True, the setting was successful (though False doesn't necessarily mean it failed). **Note: Mode changes are only effective until power cycle. After power loss, the mode reverts to the previous setting**

```python
if MotorControl1.switchControlMode(Motor1,Control_Type.POS_VEL):
    print("switch POS_VEL success")
if MotorControl1.switchControlMode(Motor2,Control_Type.VEL):
    print("switch VEL success")
```

**To persist the motor control mode, you must save parameters**

#### 6.2 Save Parameters

By default, motor mode modifications and other operations are not saved to flash. Use the following command to save to the motor's flash memory. Example below. **Note: This code saves all modifications to Motor1's flash, and must be done in DISABLED mode**. The function has automatic disable code internally to prevent the motor from being unable to save parameters while enabled.

```python
MotorControl1.save_motor_param(Motor1)
```

#### 6.3 Read Internal Register Parameters

Many internal register parameters can be read via CAN. See the DM manual for the complete parameter list. Readable parameters are all defined in the `DM_variable` enum class. Use `read_motor_param` to read:

```python
print("PMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor1,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.TMAX))
print("Motor2:")
print("PMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor2,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.TMAX))
```

After reading parameters, the current values are also stored in the corresponding motor class and can be retrieved using the `getParam` function:

```python
print("PMAX",Motor1.getParam(DM_variable.PMAX))
```

#### 6.4 Modify Internal Register Parameters

Some internal registers support modification, others are read-only (cannot be modified). Call the `change_motor_param` function to modify internal register values. Like reading registers above, the register values are also synchronized to the motor object's internal values and can be read using `Motor1.getParam`.

**Note: These internal register parameter modifications will revert to previous values after power cycle and are NOT saved**

```python
if MotorControl1.change_motor_param(Motor1,DM_variable.KP_APR,54):
   print("write success")
```

