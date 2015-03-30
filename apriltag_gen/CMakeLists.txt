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

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(gtest_${PROJECT_NAME}
        test/test_apriltag_gen.cpp
        )
    target_link_libraries(
        gtest_${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        )
endif()


