cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ros)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -Wall")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp dynamic_reconfigure message_generation
    image_transport cv_bridge image_geometry
    apriltag_mit apriltag_umich apriltag_msgs
    )

generate_dynamic_reconfigure_options(cfg/ApriltagDyn.cfg)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/apriltag_detector.cpp
    src/apriltag_detection.cpp
    )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
target_link_libraries(apriltag_detector_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

add_executable(playground test/playground.cpp)
target_link_libraries(playground
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

add_dependencies(apriltag_detector_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(gtest_apriltag_ros
        test/gtest_apriltag_ros.cpp
        test/test_timer.cpp
        test/test_apriltag_detector.cpp
        )
    target_link_libraries(
        gtest_apriltag_ros
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        )
endif()
