# Keyboard Control Guide

## Overview

This guide explains how to use keyboard keys to control Damiao motors in real-time on Linux.

## File Description

### 1. `keyboard_control.h`
This is a keyboard input processing library that provides:
- Non-blocking keyboard input reading
- Independent input thread processing
- Saving and restoring original terminal settings
- Key detection and buffer clearing functions

**Main Features:**
- `start()` - Start keyboard input listening
- `stop()` - Stop keyboard input listening
- `getKey()` - Get the last pressed key
- `clearKey()` - Clear the key buffer
- `isKeyPressed(char key)` - Check if a specific key was pressed

### 2. `test_keyboard_control.cpp`
This is a complete keyboard control example program.

## Keyboard Control Keys

| Key | Function | Description |
|------|------|------|
| W/w | Increase speed | Increases 5 rad/s per press |
| S/s | Decrease speed | Decreases 5 rad/s per press |
| A/a | Reverse rotation | Rotate in reverse at maximum speed |
| D/d | Forward rotation | Rotate forward at maximum speed |
| R/r | Reset speed | Set speed to 0 |
| Q/q | Enable motor | Turn on motor |
| E/e | Disable motor | Turn off motor |
| P/p | Print status | Display detailed motor status |
| ESC | Exit program | Safely close the program |

## Compilation Method

```bash
# Compile the keyboard control version
g++ -fdiagnostics-color=always -g test_keyboard_control.cpp -o test_keyboard_control -pthread

# Or using CMake (if the project uses CMake)
cmake ..
make
```

## Run Program

```bash
# Run the keyboard control program
./test_keyboard_control

# Or using sudo (if extra permissions are needed)
sudo ./test_keyboard_control
```

## Usage Example Workflow

1. **Start the program**
   ```
   $ ./test_keyboard_control
   ```

2. **View initialization information**
   - Program initializes serial port connection
   - Switch to speed control mode
   - Start keyboard control interface

3. **Control the motor**
   - Press `W` to increase speed
   - Press `D` to rotate forward
   - Press `P` to view motor status
   - Press `E` to close motor

4. **Exit the program**
   - Press `ESC` to exit safely
   - Program will automatically disable the motor

## Code Modification Suggestions

### If you want to add keyboard control to existing code:

**Step 1: Include header file**
```cpp
#include "keyboard_control.h"
```

**Step 2: Create keyboard object**
```cpp
KeyboardControl keyboard;
keyboard.start();  // Start keyboard listening
```

**Step 3: Handle key presses in main loop**
```cpp
while(1) {
    char key = keyboard.getKey();
    if (key == 'w') {
        velocity += 5;  // Increase speed
    }
    keyboard.clearKey();
    
    // Execute motor control
    dm.control_vel(M1, velocity);
    
    usleep(50000);  // 50ms delay
}
```

**Step 4: Clean up resources**
```cpp
keyboard.stop();  // Stop keyboard listening
```

Modify global variables:
```cpp
const float velocity_step = 10.0f;  // Change to 10 rad/s
const float max_velocity = 150.0f;   // Change to 150 rad/s
```

