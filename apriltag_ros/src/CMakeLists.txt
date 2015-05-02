add_library(${PROJECT_NAME}
    apriltag_detector.cpp
    apriltag_detection.cpp
    apriltag_map.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
    )

add_executable(apriltag_detector_node apriltag_detector_node.cpp)
target_link_libraries(apriltag_detector_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )
