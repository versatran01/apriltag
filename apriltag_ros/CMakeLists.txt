cmake_minimum_required(VERSION 3.5)
project(apriltag_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(catkin REQUIRED
             COMPONENTS dynamic_reconfigure
                        message_generation
                        image_transport
                        image_geometry
                        cv_bridge
                        apriltag
                        apriltag_msgs)

generate_dynamic_reconfigure_options(cfg/ApriltagDetectorDyn.cfg)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               ${PROJECT_NAME}
               CATKIN_DEPENDS
               dynamic_reconfigure
               message_runtime
               image_transport
               image_geometry
               cv_bridge
               apriltag
               apriltag_msgs)

add_library(${PROJECT_NAME} src/apriltag_detector.cpp)
target_include_directories(${PROJECT_NAME}
                           PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
target_link_libraries(apriltag_detector_node
                      PRIVATE ${PROJECT_NAME} Boost::timer)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} apriltag_detector_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
