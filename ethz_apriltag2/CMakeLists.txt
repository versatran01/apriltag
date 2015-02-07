cmake_minimum_required(VERSION 2.8)
project(ethz_apriltag2)

set(USE_ROS TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    )

if(USE_ROS)
    find_package(catkin REQUIRED)
    catkin_package(
        DEPENDS Eigen OpenCV
        CATKIN_DEPENDS
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        )
else()
    message(STATUS "Source:  " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary:  " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
endif()

file(GLOB SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
