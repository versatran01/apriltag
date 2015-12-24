add_library(${PROJECT_NAME} apriltag_detector.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(apriltag_detector_node apriltag_detector_node.cpp)
target_link_libraries(apriltag_detector_node ${PROJECT_NAME})

add_executable(apriltag_detect apriltag_detect.cpp)
target_link_libraries(apriltag_detect ${PROJECT_NAME})
