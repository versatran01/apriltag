cmake_minimum_required(VERSION 2.8.3)
project(apriltag_mit)

set(USE_ROS TRUE)
include(cmake/apriltag_mit-extras.cmake)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    )

if(USE_ROS)
    find_package(catkin REQUIRED COMPONENTS roslib)
    catkin_package(
        DEPENDS Eigen OpenCV
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CFG_EXTRAS apriltag_mit-extras.cmake
        )
else()
    message(STATUS "Source:  " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary:  " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
endif()

file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} SHARED ${CC_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    include_directories(${catkin_INCLUDE_DIRS})
    catkin_add_gtest(test_apriltag_mit test/test_apriltag_mit.cpp)
    target_link_libraries(test_apriltag_mit
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
endif()
