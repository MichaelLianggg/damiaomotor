#ifndef KEYBOARD_CONTROL_H
#define KEYBOARD_CONTROL_H

#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <atomic>

class KeyboardControl
{
private:
    struct termios original_settings, new_settings;
    std::atomic<char> last_key{'\0'};
    std::atomic<bool> running{false};
    std::thread input_thread;

    // Store original terminal settings
    void save_terminal_settings()
    {
        tcgetattr(STDIN_FILENO, &original_settings);
        new_settings = original_settings;
    }

    // Restore original terminal settings
    void restore_terminal_settings()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_settings);
    }

    // Enable non-blocking input
    void enable_non_blocking()
    {
        new_settings.c_lflag &= ~(ICANON | ECHO);
        new_settings.c_cc[VMIN] = 0;
        new_settings.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    }

    // Thread function for reading keyboard input
    void input_loop()
    {
        char ch;
        while (running)
        {
            if (read(STDIN_FILENO, &ch, 1) > 0)
            {
                last_key = ch;
            }
            usleep(10000); // 10ms delay
        }
    }

public:
    KeyboardControl()
    {
        save_terminal_settings();
        enable_non_blocking();
    }

    ~KeyboardControl()
    {
        stop();
        restore_terminal_settings();
    }

    // Start the keyboard input thread
    void start()
    {
        if (!running)
        {
            running = true;
            input_thread = std::thread(&KeyboardControl::input_loop, this);
        }
    }

    // Stop the keyboard input thread
    void stop()
    {
        running = false;
        if (input_thread.joinable())
        {
            input_thread.join();
        }
    }

    // Get the last pressed key (non-blocking)
    char getKey()
    {
        return last_key;
    }

    // Clear the key buffer
    void clearKey()
    {
        last_key = '\0';
    }

    // Check if a specific key is pressed
    bool isKeyPressed(char key)
    {
        return last_key == key;
    }

    // Print control instructions
    static void printInstructions()
    {
    std::cout << "\n=== Keyboard Control Instructions (MIT Mode) ===" << std::endl;
        std::cout << "w/W - Increase target position (+0.5 rad)" << std::endl;
        std::cout << "s/S - Decrease target position (-0.5 rad)" << std::endl;
        std::cout << "a/A - Set to minimum position (-12.5 rad)" << std::endl;
        std::cout << "d/D - Set to maximum position (+12.5 rad)" << std::endl;
        std::cout << "r/R - Reset target position to 0 rad" << std::endl;
        std::cout << "j/J - Increase velocity parameter (+100)" << std::endl;
        std::cout << "k/K - Decrease velocity parameter (-100)" << std::endl;
        std::cout << "i/I - Increase torque limit (+500)" << std::endl;
        std::cout << "l/L - Decrease torque limit (-500)" << std::endl;
        std::cout << "q/Q - Enable motor" << std::endl;
        std::cout << "e/E - Disable motor" << std::endl;
        std::cout << "p/P - Print detailed motor status" << std::endl;
    std::cout << "+ / - : Increase / Decrease KP (by 1.0)" << std::endl;
    std::cout << "[ / ] : Decrease / Increase KD (by 0.05)" << std::endl;
        std::cout << "ESC - Exit program" << std::endl;
        std::cout << "============================================================\n" << std::endl;
    }
};

#endif // KEYBOARD_CONTROL_H
