cmake_minimum_required(VERSION 2.8)
project(apriltag_umich)

set(USE_ROS TRUE)
add_definitions(-std=gnu99 -fPIC)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package (Threads)

if(USE_ROS)
    find_package(catkin REQUIRED)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        )
else()
    message(STATUS "Source: " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary: " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
endif()

#find_package(OpenCV REQUIRED)
include_directories(
    include
    include/apriltag
    include/apriltag/common
    )

file(GLOB C_FILES
    ${PROJECT_SOURCE_DIR}/src/*.c
    ${PROJECT_SOURCE_DIR}/src/common/*.c)
list(REMOVE_ITEM C_FILES ${PROJECT_SOURCE_DIR}/src/apriltag_demo.c)

add_library(${PROJECT_NAME} SHARED ${C_FILES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
