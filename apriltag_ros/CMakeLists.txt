cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ros)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -Wall")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp dynamic_reconfigure message_generation
    image_transport cv_bridge image_geometry
    apriltag_mit apriltag_umich apriltag_msgs
    sv_base
    )

generate_dynamic_reconfigure_options(cfg/ApriltagDyn.cfg)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_subdirectory(src)

add_dependencies(apriltag_detector_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )

add_subdirectory(test)
