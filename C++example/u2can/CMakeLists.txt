cmake_minimum_required(VERSION 3.5)

project (DM_Linux_Drive)

# Enable C++11 standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add threading support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Original test program
set(SOURCES_DAMIAO
    damiao.h
    SerialPort.h
    test_damiao.cpp
)

# Keyboard control program (Force-Position Mode)
set(SOURCES_KEYBOARD
    damiao.h
    SerialPort.h
    keyboard_control.h
    test_keyboard_control.cpp
)

# Build both executables
add_executable(DM_Linux_Drive ${SOURCES_DAMIAO})
add_executable(test_keyboard_control ${SOURCES_KEYBOARD})
add_executable(force_position_control ${SOURCES_KEYBOARD})

# Link pthread for keyboard control
target_link_libraries(test_keyboard_control PRIVATE pthread)
target_link_libraries(force_position_control PRIVATE pthread)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Add custom targets for easier use
add_custom_target(run_original
    COMMAND ./DM_Linux_Drive
    DEPENDS DM_Linux_Drive
    COMMENT "Running original test program..."
)

add_custom_target(run_keyboard
    COMMAND ./test_keyboard_control
    DEPENDS test_keyboard_control
    COMMENT "Running keyboard control program..."
)

add_custom_target(run_force_position
    COMMAND ./force_position_control
    DEPENDS force_position_control
    COMMENT "Running force-position control program..."
)
