cmake_minimum_required(VERSION 2.8.3)
project(apriltag_gen)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -Wall")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

catkin_package()

add_executable(test_apriltag_gen test/test_apriltag_gen.cpp)
target_link_libraries(test_apriltag_gen ${OpenCV_LIBRARIES})

