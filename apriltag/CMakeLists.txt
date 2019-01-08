cmake_minimum_required(VERSION 3.5)
project(apriltag)

set(CMAKE_CXX_STANDARD 14)

find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS roslib)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include apriltags LIBRARIES ${PROJECT_NAME})

file(GLOB C_FILES ${PROJECT_SOURCE_DIR}/apriltags/*.c
          ${PROJECT_SOURCE_DIR}/apriltags/common/*.c)
add_library(${PROJECT_NAME} SHARED ${C_FILES})
target_include_directories(${PROJECT_NAME}
                           PUBLIC apriltags
                           PRIVATE apriltags/common)
target_link_libraries(${PROJECT_NAME} PUBLIC ${CMAKE_THREAD_LIBS_INIT} m)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY apriltags/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h")
