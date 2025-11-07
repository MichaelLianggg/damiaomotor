# DAMIAO | DAMIAO Technology

DM-MC01 Demo for driving multiple DM-J4310 modules

![image-20240322094002609](C:/Users/disno/AppData/Roaming/Typora/typora-user-images/image-20240322094002609.png)

User Manual V1.0 2023-12-08

## Revision History

| No. | Date       | Version | Description |
| :--:| :--------: | :-----: | :---------: |
|  1  | 2023-12-08 |  V1.0   | Initial release |

## Contents

[TOC]

<div style="page-break-after:always"></div>

## 1. Demo usage

The demo implements a layered structure separating application and drivers to ease porting.

![image-20231208162813672](C:\Users\disno\AppData\Roaming\Typora\typora-user-images\image-20231208162813672.png)

### 1.1 LCD display

1. Motor status: **[READY]** motor ready	**[START]** motor running	**[ERROR]** motor alarm
2. Motor ID: select motor ID number
3. Control modes: **[MIT.]** MIT mode     **[Pos.]** Position+velocity mode     **[Vel.]** Velocity mode
4. Temperature feedback display
5. Motor parameter settings and feedback values

### 1.2 Key behaviour

user key: short press to enable motor, long press to disable motor (this key is on DM-MC01)

6. mid_key: short press to enter parameter setting, long press to confirm

7. up_key: move selection up (in parameter edit mode, increases value)

8. down_key: move selection down (in parameter edit mode, decreases value)
9. left_key: long press to clear motor error parameters
10. right_key: long press to clear set parameters

### 1.3 Motor ID configuration

When the main board controls multiple motors on the same CAN bus, ensure each motor's **CAN ID** and **Master ID** are unique. Use the DAMIAO debug tool to configure IDs.

**Note: Master ID must not conflict with any data IDs returned by the motor. On the same bus, CAN ID and Master ID must be unique and not duplicate each other.**

![image-20231208150846495](C:\Users\disno\AppData\Roaming\Typora\typora-user-images\image-20231208150846495.png)

**Example CAN1 (two motors):**

- MOTOR1: CAN ID 0x01  Master ID 0x00
- MOTOR3: CAN ID 0x02  Master ID 0x00

**Example CAN2 (one motor):**

- MOTOR2: CAN ID 0x01  Master ID 0x00


## 2. Adding a motor to the demo

To add a motor to the demo, modify the `dm4310_ctrl` and `display` modules.

### 2.1 Adding a motor

To add Motor4, extend the enum below:

```c
typedef enum
{
	Motor1,
	Motor2,
	Motor3,
	...
	num
} motor_num;
```

Then add initialization for the new motor, including its ID and default control mode:

```c
void dm4310_motor_init(void)
{
	// initialize motor data structures
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));

	// Motor1 settings
	motor[Motor1].id = 1;
	motor[Motor1].ctrl.mode = 2;    // 0: MIT   1: Position+Velocity   2: Velocity
	motor[Motor1].cmd.mode = 2;

	// Motor2 settings
	motor[Motor2].id = 1;
	motor[Motor2].ctrl.mode = 2;
	motor[Motor2].cmd.mode = 2;

	// Motor3 settings
	motor[Motor3].id = 2;
	motor[Motor3].ctrl.mode = 2;
	motor[Motor3].cmd.mode = 2;
    
	...
}
```

Adjust the control switch cases in `dm4310_ctrl.c` accordingly.

### 2.2 Receiving data for new motor

Differentiate motors by the received CAN ID. If you add a motor, extend the switch handling the received ID.

```c
void can1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan1, &rec_id, rx_data);
	switch (rec_id)
	{
		case 0: dm4310_fbdata(&motor[Motor1], rx_data); break;
		case 1: dm4310_fbdata(&motor[Motor3], rx_data); break;
		...
	}
}
```

### 2.3 Updating display for new motor

Update `display.c` to show Motor4 information:

```c
void motor_id_display(void)
{
	switch(motor_id)
	{
		case 1:
			motor_start_display(&motor[Motor1]);
			LCD_ShowString(200, 10,(uint8_t *)"MOTOR1", WHITE, BLACK, 24, 0);
			motor_para_display(&motor[Motor1]);
			break;
		case 2:
			motor_start_display(&motor[Motor2]);
			LCD_ShowString(200, 10,(uint8_t *)"MOTOR2", WHITE, BLACK, 24, 0);
			motor_para_display(&motor[Motor2]);
			break;
		case 3:
			motor_start_display(&motor[Motor3]);
			LCD_ShowString(200, 10,(uint8_t *)"MOTOR3", WHITE, BLACK, 24, 0);
			motor_para_display(&motor[Motor3]);
			break;
		...
	}
}
```

## 3. Project structure

```bash
CtrBoard-DM4310
├─ .mxproject                      # STM32CubeMX project files
├─ CtrBoard.ioc                    # STM32CubeMX configuration
├─ Core                            # HAL peripheral configuration
│    ├─ Inc
│    └─ Src
├─ Drivers                         # Low-level drivers
├─ MDK-ARM                         # Keil startup files
├─ README.md
└─ User                            # User code
	   ├─ bsp
	   │    ├─ adc_bsp.c           # board-level ADC configuration
	   │    ├─ adc_bsp.h           # board-level ADC header
	   │    ├─ can_bsp.c           # board-level CAN configuration
	   │    ├─ can_bsp.h           # board-level CAN header
	   │    ├─ key_bsp.c           # board-level key configuration
	   │    ├─ key_bsp.h           # board-level key header
	   │    ├─ led_bsp.c           # board-level LED configuration
	   │    └─ led_bsp.h           # board-level LED header
	   ├─ driver
	   │    ├─ adc_driver.c        # ADC driver
	   │    ├─ adc_driver.h        # ADC driver header
	   │    ├─ can_driver.c        # CAN driver
	   │    ├─ can_driver.h        # CAN driver header
	   │    ├─ drivers.h           # drivers header
	   │    ├─ key_driver.c        # key driver
	   │    ├─ key_driver.h        # key driver header
	   │    ├─ led_driver.c        # LED driver
	   │    └─ led_driver.h        # LED driver header
	   ├─ module
	   │   ├─ adc_modlue.c         # ADC module
	   │   ├─ adc_modlue.h         # ADC module header
	   │   ├─ display.c           # display module
	   │   ├─ display.h           # display header
	   │   ├─ key_modlue.c         # key module
	   │   ├─ key_modlue.h         # key module header
	   │   ├─ lcd.c               # LCD implementation
	   │   ├─ lcd.h               # LCD header
	   │   ├─ lcdfont.h           # LCD font header
	   │   ├─ mcu_config.c        # MCU configuration
	   │   ├─ mcu_config.h        # MCU config header
	   │   ├─ pic.h               # images header
	   │   ├─ scheduler.c         # scheduler
	   │   ├─ scheduler.h         # scheduler header
	   │   ├─ task.c              # tasks
	   │   └─ task.h              # task header
	   └─ motor
		   ├─ dm4310_ctrl.c       # DM4310 motor control
		   ├─ dm4310_ctrl.h       # DM4310 control header
		   ├─ dm4310_drv.c        # DM4310 driver
		   └─ dm4310_drv.h        # DM4310 driver header
```
