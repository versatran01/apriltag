cmake_minimum_required(VERSION 3.5)
project(apriltag_umich)

set(USE_ROS TRUE)
set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)
include_directories(include src)

if(USE_ROS)
  find_package(catkin REQUIRED COMPONENTS roslib)
  include_directories(${catkin_INCLUDE_DIRS})

  catkin_package(INCLUDE_DIRS include src LIBRARIES ${PROJECT_NAME})

  file(GLOB C_FILES ${PROJECT_SOURCE_DIR}/src/*.c
            ${PROJECT_SOURCE_DIR}/src/common/*.c)
  add_library(${PROJECT_NAME} SHARED ${C_FILES})
  target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} m)

  if(CATKIN_ENABLE_TESTING)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    catkin_add_gtest(test_apriltag_umich test/test_apriltag_umich.cpp)
    target_link_libraries(test_apriltag_umich
                          ${PROJECT_NAME}
                          ${OpenCV_LIBRARIES}
                          ${catkin_LIBRARIES})
  endif()

  install(TARGETS ${PROJECT_NAME}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY src/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
          FILES_MATCHING
          PATTERN "*.h")
else()
  message(STATUS "Source: " ${PROJECT_SOURCE_DIR})
  message(STATUS "Binary: " ${PROJECT_BINARY_DIR})
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
  link_directories(${PROJECT_SOURCE_DIR}/lib)
endif()
