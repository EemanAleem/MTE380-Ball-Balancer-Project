cmake_minimum_required(VERSION 3.16)
project(ball_balancer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# If the user didn't pass -DOpenCV_DIR=... but has an OpenCV_DIR env variable, use that
if(NOT OpenCV_DIR AND DEFINED ENV{OpenCV_DIR})
    set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "Directory containing OpenCVConfig.cmake")
endif()
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
    # OpenCVConfig.cmake defines OpenCV_ARCH (x64) and OpenCV_RUNTIME (vc16),
    # so we derive the bin folder from OpenCV_DIR instead of hardcoding C:/opencv
    file(GLOB OPENCV_DLLS "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world*.dll")
    if(OPENCV_DLLS)
        add_custom_command(TARGET ball_balancer POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${OPENCV_DLLS}
                    $<TARGET_FILE_DIR:ball_balancer>
        )
    else()
        message(WARNING "No opencv_world*.dll found. Add OpenCV's bin folder to PATH as 'OpenCV_DIR' or copy the DLLs manually.")
    endif()
endif()
