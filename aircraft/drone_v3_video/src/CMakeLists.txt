cmake_minimum_required(VERSION 3.16)
project(DroneCameraPipeline LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# === Автоматическое определение хоста/таргета ===
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(STATUS "🟢 Native ARM64 build (for Pi Zero 2W)")
    set(TARGET_ARCH "arm64")
else()
    message(STATUS "🔵 Cross-compiling for ARM64 from x86_64")
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR aarch64)
    set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
    set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(TARGET_ARCH "aarch64")
endif()

# === Зависимости ===
#find_package(OpenCV REQUIRED)    # ← Закомментируй
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# === Исполняемый файл ===
add_executable(camera_pipeline src/main.cpp src/camera_interface.cpp src/uart_telemetry.cpp)
#target_include_directories(camera_pipeline PRIVATE ${LIBCAMERA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_pipeline PRIVATE ${LIBCAMERA_LIBRARIES} ${OpenCV_LIBS} pthread)
target_link_libraries(drone_camera PRIVATE ${LIBCAMERA_LIBRARIES} pthread)

# === Оптимизации для Pi Zero 2W ===
target_compile_options(camera_pipeline PRIVATE -O2 -mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard)
install(TARGETS camera_pipeline DESTINATION bin)

