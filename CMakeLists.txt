cmake_minimum_required(VERSION 3.16)
project(ball_balancer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(ball_balancer
    main.cpp
    ball_tracker.cpp
    pid.cpp
    pid_ui.cpp
    ik.cpp
    math.cpp
    servo_link.cpp
    serial_port.cpp
    text_render.cpp
)
target_include_directories(ball_balancer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ball_balancer PRIVATE ${OpenCV_LIBS})