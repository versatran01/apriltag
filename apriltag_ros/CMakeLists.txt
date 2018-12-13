cmake_minimum_required(VERSION 3.5)
project(apriltag_ros)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(catkin REQUIRED
             COMPONENTS roscpp
                        dynamic_reconfigure
                        message_generation
                        image_transport
                        image_geometry
                        cv_bridge
                        tf2_ros
                        tf2_geometry_msgs
                        apriltag_mit
                        apriltag_umich
                        apriltag_msgs)

generate_dynamic_reconfigure_options(cfg/ApriltagDetectorDyn.cfg)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               ${PROJECT_NAME}
               CATKIN_DEPENDS
               roscpp
               dynamic_reconfigure
               message_runtime
               image_transport
               image_geometry
               cv_bridge
               tf2_ros
               tf2_geometry_msgs
               apriltag_mit
               apriltag_umich
               apriltag_msgs)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/apriltag_detector.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
target_link_libraries(apriltag_detector_node ${PROJECT_NAME}
                      ${OpenCV_LIBRARIES})

add_executable(apriltag_detect src/apriltag_detect.cpp)
target_link_libraries(apriltag_detect ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(apriltag_pose_estimator src/apriltag_pose_estimator.cpp)
target_link_libraries(apriltag_pose_estimator ${PROJECT_NAME}
                      ${OpenCV_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
                apriltag_detector_node
                apriltag_detect
                apriltag_pose_estimator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# if(CATKIN_ENABLE_TESTING) catkin_add_gtest(test_apriltag_detector
# test/test_apriltag_detector.cpp) target_link_libraries(test_apriltag_detector
# ${PROJECT_NAME}) endif()
