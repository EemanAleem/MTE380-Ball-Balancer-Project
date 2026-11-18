cmake_minimum_required(VERSION 3.16)
project(ball_balancer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "C:/Users/khush/opencv/build")
find_package(OpenCV REQUIRED)

add_executable(ball_balancer
    src/main.cpp
    src/ball_tracker.cpp
    src/ik.cpp
    src/math.cpp
    src/pid.cpp
    src/pid_ui.cpp
    src/serial_port.cpp
    src/servo_link.cpp
)

target_include_directories(ball_balancer PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ball_balancer PRIVATE ${OpenCV_LIBS})

if (WIN32)
    file(GLOB OPENCV_DLLS "C:/Users/khush/opencv/build/x64/vc16/bin/opencv_world*.dll")
    add_custom_command(TARGET ball_balancer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENCV_DLLS} $<TARGET_FILE_DIR:ball_balancer>
    )
endif()