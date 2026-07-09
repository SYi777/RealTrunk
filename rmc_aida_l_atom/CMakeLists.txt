#cmake最小需要版本
cmake_minimum_required(VERSION 3.5)
#项目名字
project(atom)

# 设置 C 标准
set(CMAKE_C_STANDARD 11)  # 设置 C 标准为 C11

# 设置源文件
set(SOURCES
        src/serial_port.cpp
        src/data_filter.cpp
        src/data_broadcast.cpp
        src/arm_teleoperation.cpp
        src/lift_agv_teleoperation.cpp
        src/woosh_robot_control.cpp
        src/inih/ini.c
        src/inih/INIReader.cpp
        src/main_teleoperation.cpp
        src/io_state_listener.cpp
        src/agv_controller.cpp
        src/lift_controller.cpp
        src/master_arm_listener.cpp
        src/slave_arm_controller.cpp
        src/tcp_service.cpp
)

# 添加可执行文件
add_executable(atom ${SOURCES})

# 定义库和头文件的路径
set(ROBOTIC_ARM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Robotic_Arm")
set(ROBOTIC_ARM_LIB_DIR "${ROBOTIC_ARM_DIR}/lib")
set(ROBOTIC_ARM_INCLUDE_DIR "${ROBOTIC_ARM_DIR}/include")
# set(INIH_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/inih/include")
set(PROJECT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(DEP_LIB
        ${CMAKE_SOURCE_DIR}/lib/linux/libwoosh_robot.so
        ${CMAKE_SOURCE_DIR}/lib/linux/libprotobuf.so.32
        pthread)

# 查找库
find_library(RMAN_API_LIB NAMES api_cpp PATHS "${ROBOTIC_ARM_LIB_DIR}")

if(NOT RMAN_API_LIB)
    message(FATAL_ERROR "api_cpp library not found")
else()
    message(STATUS "RMAN_API_LIB: ${RMAN_API_LIB}")

    # 链接库
    target_link_libraries(atom ${RMAN_API_LIB} ${DEP_LIB} -ljsoncpp)

    # 包含目录
    target_include_directories(atom PUBLIC "${ROBOTIC_ARM_INCLUDE_DIR}" "${PROJECT_INCLUDE_DIR}")

    # 如果是 Windows 平台，拷贝 DLL 文件到可执行文件的目录
    if(WIN32)
        add_custom_command(TARGET atom POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${ROBOTIC_ARM_LIB_DIR}/api_cpp.dll"
                $<TARGET_FILE_DIR:atom>)
    endif()

    # 如果是 UNIX 平台
    if(UNIX)
        add_custom_command(TARGET atom POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/Robotic_Arm/lib/libapi_cpp.so"
                $<TARGET_FILE_DIR:atom>)
    endif()

endif()

