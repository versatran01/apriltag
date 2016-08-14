cmake_minimum_required(VERSION 2.8.3)
project(apriltag_mit)

set(USE_ROS TRUE)
include(cmake/apriltag_mit-extras.cmake)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(USE_ROS)
    find_package(catkin REQUIRED COMPONENTS roslib cmake_modules)
    find_package(Eigen3 REQUIRED)
    include_directories(${Eigen_INCLUDE_DIRS})
    catkin_package(
        DEPENDS Eigen3 OpenCV
        INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME}
        CFG_EXTRAS apriltag_mit-extras.cmake
        )
else()
    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIR})
    message(STATUS "Source:  " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary:  " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
endif()

find_package(OpenCV REQUIRED)
include_directories(include
    ${OpenCV_INCLUDE_DIRS}
    )

include_directories(include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} SHARED ${CC_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    include_directories(${catkin_INCLUDE_DIRS})
    catkin_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
    target_link_libraries(test_${PROJECT_NAME}
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
endif()
