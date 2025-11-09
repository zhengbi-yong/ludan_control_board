cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(MCU_FLAGS
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv5-d16
  -mfloat-abi=hard
)
set(COMMON_FLAGS
  -ffunction-sections
  -fdata-sections
  -fno-builtin
  -g3
)
add_compile_options(${MCU_FLAGS} ${COMMON_FLAGS})
add_link_options(${MCU_FLAGS} -Wl,--gc-sections)
# 宏定义
add_compile_definitions(
  STM32H723xx
  USE_HAL_DRIVER
  HSE_VALUE=25000000U        # 按你的板改
  LSE_VALUE=32768U
)
# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ludan_control_board)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    User/App/fdcan1_task.c
    User/App/ChassisL_task.c
    User/App/vbus_check.c
    User/App/observe_task.c
    User/Devices/DM_Motor/motor_config.c
    User/Bsp/bsp_dwt.c
    User/Bsp/bsp_PWM.c
    User/Bsp/bsp_usart1.c
    User/Bsp/can_bsp.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    # User/Algorithm/EKF
    # User/Algorithm/kalman
    # User/Algorithm/mahony
    # User/Algorithm/PID
    # User/Algorithm/VMC
    User/App
    User/Bsp
    User/Controller
    User/Devices/BMI088
    User/Devices/DM_Motor
    User/Lib
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)
