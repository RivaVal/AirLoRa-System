cmake_minimum_required(VERSION 3.16)
project(DroneCameraPipeline LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Автоопределение: кросс-компиляция или нативная сборка
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(STATUS "🟢 Native ARM64 build")
else()
    message(STATUS "🔵 Cross-compiling for ARM64 (x86_64 host)")
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR aarch64)
    set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
    set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

add_executable(drone_camera main.cpp uart_telemetry.cpp)
target_include_directories(drone_camera PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
target_link_libraries(drone_camera PRIVATE ${LIBCAMERA_LIBRARIES} pthread)

# Оптимизации под Cortex-A53 (Pi Zero 2W)
target_compile_options(drone_camera PRIVATE -O2 -mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard)
