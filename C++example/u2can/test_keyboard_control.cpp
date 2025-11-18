#include "damiao.h"
#include "keyboard_control.h"
#include "unistd.h"
#include <cmath>
#include <iomanip>

damiao::Motor M1(damiao::DM4310, 0x02, 0x52);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);
KeyboardControl keyboard;

float current_position = 0.0f;
float target_position = 0.0f;
const float position_step = 0.5f;  // Position step in radians
const float max_position = 12.5f;  // Max position (DM4310 limit)
const float min_position = -12.5f; // Min position
uint16_t current_velocity = 500;   // Velocity for position reaching (0-10000)
uint16_t current_torque = 2000;    // Torque limit (0-10000)
// MIT controller gains (exposed to keyboard)
float kp = 30.0f;
float kd = 0.3f;
const uint16_t velocity_step_force = 100;
const uint16_t torque_step = 500;
bool motor_enabled = false;

void print_status()
{
    std::cout << "\r[POS: " << std::setw(7) << std::fixed << std::setprecision(2) 
              << current_position << " rad | "
              << "TGT: " << std::setw(7) << target_position << " rad | "
              << "TAU: " << std::setw(5) << current_torque << " | "
              << "VEL: " << std::setw(5) << current_velocity << " | "
              << "ACT: " << std::setw(6) << M1.Get_tau() << " Nm | "
              << "Status: " << (motor_enabled ? "ENABLED " : "DISABLED") << " ]   " << std::flush;
}

void handle_keyboard_input()
{
    char key = keyboard.getKey();

    if (key == '\0')
        return; // No key pressed

    switch (key)
    {
    case 'w':
    case 'W':
        // Increase target position (clockwise)
        target_position += position_step;
        if (target_position > max_position)
            target_position = max_position;
        std::cout << "\n[INFO] Target position increased to: " << target_position << " rad" << std::endl;
        break;

    case 's':
    case 'S':
        // Decrease target position (counter-clockwise)
        target_position -= position_step;
        if (target_position < min_position)
            target_position = min_position;
        std::cout << "\n[INFO] Target position decreased to: " << target_position << " rad" << std::endl;
        break;

    case 'a':
    case 'A':
        // Set to minimum position
        target_position = min_position;
        std::cout << "\n[INFO] Target position set to minimum: " << target_position << " rad" << std::endl;
        break;

    case 'd':
    case 'D':
        // Set to maximum position
        target_position = max_position;
        std::cout << "\n[INFO] Target position set to maximum: " << target_position << " rad" << std::endl;
        break;

    case 'r':
    case 'R':
        // Reset target position to zero
        target_position = 0.0f;
        std::cout << "\n[INFO] Target position reset to 0 rad" << std::endl;
        break;

    case 'j':
    case 'J':
        // Increase velocity parameter for reaching position
        current_velocity += velocity_step_force;
        if (current_velocity > 10000)
            current_velocity = 10000;
        std::cout << "\n[INFO] Velocity parameter increased to: " << current_velocity << std::endl;
        break;

    case 'k':
    case 'K':
        // Decrease velocity parameter
        if (current_velocity > velocity_step_force)
            current_velocity -= velocity_step_force;
        else
            current_velocity = 0;
        std::cout << "\n[INFO] Velocity parameter decreased to: " << current_velocity << std::endl;
        break;

    case 'i':
    case 'I':
        // Increase torque limit
        current_torque += torque_step;
        if (current_torque > 10000)
            current_torque = 10000;
        std::cout << "\n[INFO] Torque limit increased to: " << current_torque << std::endl;
        break;

    case 'l':
    case 'L':
        // Decrease torque limit
        if (current_torque > torque_step)
            current_torque -= torque_step;
        else
            current_torque = 0;
        std::cout << "\n[INFO] Torque limit decreased to: " << current_torque << std::endl;
        break;

    case 'q':
    case 'Q':
        // Enable motor
        if (!motor_enabled)
        {
            dm.enable(M1);
            motor_enabled = true;
            std::cout << "\n[INFO] Motor enabled" << std::endl;
        }
        break;

    case 'e':
    case 'E':
        // Disable motor
        if (motor_enabled)
        {
            dm.disable(M1);
            motor_enabled = false;
            target_position = 0.0f;
            std::cout << "\n[INFO] Motor disabled" << std::endl;
        }
        break;

    case 'p':
    case 'P':
        // Print detailed status
        std::cout << "\n\n=== Motor Status (MIT Mode) ===" << std::endl;
        std::cout << "Current Position: " << current_position << " rad" << std::endl;
        std::cout << "Target Position: " << target_position << " rad" << std::endl;
        std::cout << "Actual Torque: " << M1.Get_tau() << " Nm" << std::endl;
        std::cout << "Velocity Parameter: " << current_velocity << std::endl;
        std::cout << "Torque Limit: " << current_torque << std::endl;
        std::cout << "KP: " << kp << " | KD: " << kd << std::endl;
        std::cout << "Status: " << (motor_enabled ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "=========================================\n" << std::endl;
        break;

    // KP / KD tuning via keyboard
    case '+':
    case '=': // some keyboards send '=' for unshifted plus
        kp += 1.0f;
        std::cout << "\n[INFO] KP increased to: " << kp << std::endl;
        break;

    case '-':
    case '_':
        kp -= 1.0f;
        if (kp < 0.0f) kp = 0.0f;
        std::cout << "\n[INFO] KP decreased to: " << kp << std::endl;
        break;

    case ']':
        kd += 0.05f;
        std::cout << "\n[INFO] KD increased to: " << kd << std::endl;
        break;

    case '[':
        kd -= 0.05f;
        if (kd < 0.0f) kd = 0.0f;
        std::cout << "\n[INFO] KD decreased to: " << kd << std::endl;
        break;

    case 27: // ESC key
        std::cout << "\n\n[INFO] Exit command received" << std::endl;
        return;

    default:
        // Unknown key
        break;
    }

    keyboard.clearKey();
}

int main(int argc, char *argv[])
{
    try
    {
        // Initialize serial port
        serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        dm = damiao::Motor_Control(serial);

        // Add motor
        dm.addMotor(&M1);

        // Initialize motor
        std::cout << "[INFO] Initializing motor..." << std::endl;
        dm.disable(M1);
        sleep(1);

        // Switch to MIT mode (MIT_MODE)
        if (dm.switchControlMode(M1, damiao::MIT_MODE))
        {
            std::cout << "[INFO] Switched to MIT_MODE successfully" << std::endl;
        }
        else
        {
            std::cout << "[WARNING] Failed to switch to MIT_MODE" << std::endl;
        }

        dm.save_motor_param(M1);
        dm.enable(M1);
        motor_enabled = true;
        sleep(1);

        // Start keyboard control
        keyboard.start();
        KeyboardControl::printInstructions();

        // Main control loop
        std::cout << "\n[INFO] Starting keyboard control loop. Press 'ESC' to exit." << std::endl;

        while (true)
        {
            // Handle keyboard input
            char key = keyboard.getKey();
            if (key == 27) // ESC key
            {
                std::cout << "\n\n[INFO] Exiting program..." << std::endl;
                break;
            }
            handle_keyboard_input();

            // Apply MIT control command (replacing previous pos_force usage)
            if (motor_enabled)
            {
                current_position = M1.Get_Position();
                // Map the keyboard parameters to MIT's expected float inputs:
                // - kp and kd are chosen as reasonable defaults
                // - q is target position
                // - dq is mapped from current_velocity (0-10000) -> [-DQ_MAX, DQ_MAX]
                // - tau is mapped from current_torque (0-10000) -> [-TAU_MAX, TAU_MAX]
                float kp = 30.0f;
                float kd = 0.3f;
                auto limits = M1.get_limit_param();
                float dq = (static_cast<float>(current_velocity) / 10000.0f) * limits.DQ_MAX;
                float tau = (static_cast<float>(current_torque) / 10000.0f) * limits.TAU_MAX;
                dm.control_mit(M1, kp, kd, target_position, dq, tau);
            }

            // Refresh and display motor status
            dm.refresh_motor_status(M1);
            print_status();

            usleep(50000); // 50ms delay
        }

        // Cleanup
        std::cout << "\n[INFO] Disabling motor..." << std::endl;
        dm.disable(M1);
        sleep(1);
        keyboard.stop();

        std::cout << "[INFO] Program terminated successfully" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
