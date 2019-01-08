cmake_minimum_required(VERSION 3.5)
project(apriltag)

set(CMAKE_CXX_STANDARD 14)

find_package(Threads REQUIRED)
include_directories(include src)

find_package(catkin REQUIRED COMPONENTS roslib)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include src LIBRARIES ${PROJECT_NAME})

file(GLOB C_FILES ${PROJECT_SOURCE_DIR}/src/*.c
          ${PROJECT_SOURCE_DIR}/src/common/*.c)
add_library(${PROJECT_NAME} SHARED ${C_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC src PRIVATE src/common)
target_link_libraries(${PROJECT_NAME} PUBLIC ${CMAKE_THREAD_LIBS_INIT} m)

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
