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

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(gtest_apriltag_ros
        test/gtest_apriltag_ros.cpp
        test/test_timer.cpp)
    target_link_libraries(gtest_apriltag_ros ${catkin_LIBRARIES})
endif()

# apriltag_ros library
#add_library(${PROJECT_NAME}
#src/mapper/mapper.cpp
#src/mapper/tag_map.cpp
#src/utils/utils.cpp
#src/visualizer/visualizer.cpp
#src/detector/detector_node.cpp
#src/mapper/mapper_node.cpp
#)
#target_link_libraries(${PROJECT_NAME}
#${catkin_LIBRARIES}
#${Gtsam_LIBRARIES}
#)

# detector node
#add_executable(detector_node
#src/detector/detector_main.cpp
#)
#target_link_libraries(detector_node
#${PROJECT_NAME}
#)

#add_executable(mapper_node
#src/mapper/mapper_main.cpp
#)
#target_link_libraries(mapper_node
#${PROJECT_NAME}
#)

# dependencies
#add_dependencies(${PROJECT_NAME}
#${catkin_EXPORTED_TARGETS}
#${PROJECT_NAME}_gencfg
#${PROJECT_NAME}_gencpp
#${${PROJECT_NAME}_EXPORTED_TARGETS}
#)

# Map generator
#add_executable(generate_map
#src/generate_map.cpp
#src/tag_yaml.cpp
#)

#target_link_libraries(generate_map
#${catkin_LIBRARIES}
#${OpenCV_LIBRARIES}
#yaml-cpp
#)
