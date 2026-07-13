cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME JetsonExtHardwareManager)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
#
# NOTE: i2c_app.c / i2c_app.h were REMOVED here — they are replaced by the
# App/ modules below (app.c + i2c_reg.c). Leaving them in the build would
# duplicate the HAL_I2C_* callbacks and fail to link.
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/debug_console.c
        Core/Inc/debug_console.h
        Core/Inc/btn_def.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Actuation-circuit firmware (App/ modules)
        App/Src/app.c
        App/Src/actuation.c
        App/Src/app_status.c
        App/Src/comms_wdg.c
        App/Src/crc8.c
        App/Src/envelope.c
        App/Src/i2c_reg.c
        App/Src/pwm_out.c
        App/Src/rgb_led.c
        App/Src/sensor_as.c
        App/Src/telemetry.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        App/Inc
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