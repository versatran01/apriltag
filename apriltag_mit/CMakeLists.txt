cmake_minimum_required(VERSION 2.8.3)
project(apriltag_mit)

set(USE_ROS TRUE)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

if(USE_ROS)
    find_package(catkin REQUIRED COMPONENTS roslib)
    catkin_package(
        DEPENDS OpenCV EIGEN3
        INCLUDE_DIRS
        include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
        )

    include_directories(${catkin_INCLUDE_DIRS})
    file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
    add_library(${PROJECT_NAME} SHARED ${CC_FILES})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

    catkin_add_gtest(test_apriltag_mit test/test_apriltag_mit.cpp)
    target_link_libraries(test_apriltag_mit
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        )
else()
    message(STATUS "Source:  " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary:  " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)

    file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
    add_library(${PROJECT_NAME} SHARED ${CC_FILES})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()
