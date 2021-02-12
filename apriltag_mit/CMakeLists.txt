cmake_minimum_required(VERSION 3.5)
project(apriltag_mit)

set(USE_ROS1 FALSE)
set(USE_ROS2 TRUE)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)
include_directories(include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS roslib)
  catkin_package(DEPENDS
                 OpenCV
                 EIGEN3
                 INCLUDE_DIRS
                 include
                 LIBRARIES
                 ${PROJECT_NAME})

  include_directories(${catkin_INCLUDE_DIRS})
  file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
  add_library(${PROJECT_NAME} SHARED ${CC_FILES})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_apriltag_mit test/test_apriltag_mit.cpp)
    target_link_libraries(test_apriltag_mit ${catkin_LIBRARIES} ${PROJECT_NAME})
  endif()

  install(TARGETS ${PROJECT_NAME}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
elseif(USE_ROS2)
  #
  # ---------- ROS2 -----------------------
  #
  find_package(ament_cmake REQUIRED)

  include_directories(include)

  file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
  add_library(${PROJECT_NAME} SHARED ${CC_FILES})
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen opencv_core opencv_calib3d)

  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib
    INCLUDES DESTINATION include)

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME})

  ament_export_targets(export_${PROJECT_NAME})
  ament_package()
else()
  message(STATUS "Source:  " ${PROJECT_SOURCE_DIR})
  message(STATUS "Binary:  " ${PROJECT_BINARY_DIR})
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
  link_directories(${PROJECT_SOURCE_DIR}/lib)

  file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
  add_library(${PROJECT_NAME} SHARED ${CC_FILES})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()
