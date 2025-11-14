# Quick Reference

## Force-Position Hybrid Mode - Keyboard Key Cheat Sheet

### 📍 Position Control
```
W/w  → Target position +0.5 rad
S/s  → Target position -0.5 rad  
A/a  → Minimum position (-12.5 rad)
D/d  → Maximum position (+12.5 rad)
R/r  → Reset position (0 rad)
```

### ⚡ Speed Parameter (0-10000)
```
J/j  → Speed parameter +100
K/k  → Speed parameter -100
```

### 💪 Torque Limit (0-10000)
```
I/i  → Torque limit +500
L/l  → Torque limit -500
```

### 🎛️ Motor Control
```
Q/q  → Enable motor
E/e  → Disable motor
P/p  → Display detailed status
ESC  → Exit program
```

---

## Parameter Meanings

| Parameter | Range | Function |
|------|------|------|
| **Target Position** | -12.5~+12.5 rad | Specify the angle the motor should reach |
| **Speed Parameter** | 0~10000 | Control the speed of reaching the target position |
| **Torque Limit** | 0~10000 | Limit output torque to protect machinery |

---

## Workflow

```
1. Start program
   └─ Initialize motor
      └─ Switch to force-position hybrid mode
         └─ Auto enable motor
            └─ Wait for keyboard input

2. Key control
   ├─ W/S: Adjust target position
   ├─ J/K: Adjust speed parameter
   ├─ I/L: Adjust torque limit
   ├─ P: View status
   └─ Q/E: Toggle motor

3. Exit program
   └─ Press ESC
      └─ Auto disable motor
         └─ Program ends
```

---

## Typical Applications

### 🔹 Precision Positioning
```
Speed parameter: 1000-2000
Torque limit: 3000-5000
Scenario: Need precise stop and control
```

### 🔹 Normal Application
```
Speed parameter: 3000-5000
Torque limit: 5000-7000
Scenario: Balance speed and force application
```

### 🔹 Fast Movement
```
Speed parameter: 7000-10000
Torque limit: 7000-10000
Scenario: Need fast response
```

---

## Compilation Command

```bash
g++ -fdiagnostics-color=always -g test_keyboard_control.cpp \
    -o test_keyboard_control -pthread
```

## Run Command

```bash
./test_keyboard_control
# Or when permissions are required
sudo ./test_keyboard_control
```

---

## Troubleshooting

| Issue | Cause | Solution |
|------|------|----------|
| Keyboard unresponsive | Program not focused | Click terminal window |
| Motor not moving | Not enabled | Press Q key |
| Motor too fast/slow | Inappropriate speed parameter | Press J/K to adjust |
| Cannot reach target position | Insufficient torque | Press I to increase torque |

---

## Display Information Interpretation

```
[POS: 1.23 rad | TGT: 2.50 rad | TAU: 2000 | VEL: 500 | ACT: 2.45 Nm | Status: ENABLED]
 ↑              ↑              ↑         ↑        ↑               ↑
 Current pos    Target pos    Torque    Speed    Actual torque   Motor status
```



