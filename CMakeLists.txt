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
set(CMAKE_PROJECT_NAME FDR-Core)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# 告诉链接器：链接带有硬件浮点支持的 C 标准库
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# 可选优化：防止代码里没写 'f' (比如 1.2) 被当成双精度(double)导致退化为软件模拟
add_compile_options(-fsingle-precision-constant)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# 强制链接器加入 printf 的浮点数格式化支持
add_link_options(-u _printf_float)
# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Inc/motor.h
        Core/Src/motor.c
        Core/Inc/ladrc.h
        Core/Src/ladrc.c
        Core/Inc/f4_can_app.h
        Core/Src/f4_can_app.c
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
# 强制链接器加入 printf 的浮点数格式化支持

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
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
