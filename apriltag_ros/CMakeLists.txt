cmake_minimum_required(VERSION 3.5)
project(apriltag_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED core imgproc calib3d)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Threads REQUIRED)   # required by umich3
find_package(Eigen3 REQUIRED)  # required by apriltag_mit

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(apriltag_mit REQUIRED)

add_subdirectory(apriltag_umich)

add_library(${PROJECT_NAME} SHARED src/apriltag_detector.cpp)

# apparently the non-ament packages need to be added like this.
# Note: apriltag is the name of the umich apriltag package
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} apriltag opencv_core)

# specify the dependencies on ament packages
ament_target_dependencies(${PROJECT_NAME} apriltag_msgs image_transport cv_bridge apriltag_mit)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE include)

# component (like ROS1 nodelet) library 
add_library(apriltag_detector_component SHARED src/apriltag_detector_component.cpp)
target_link_libraries(apriltag_detector_component ${PROJECT_NAME} opencv_core)

# the actual node uses the same nodelet code
add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
target_link_libraries(apriltag_detector_node apriltag_detector_component ${PROJECT_NAME})

# register the component
rclcpp_components_register_nodes(apriltag_detector_component "apriltag_ros::ApriltagDetectorComponent")

install(TARGETS
  ${PROJECT_NAME}
  apriltag_detector_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
  INCLUDES include)

# must be installed under PROJECT_NAME directory or else not found on launch
install(TARGETS apriltag_detector_node
  DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

ament_package()
