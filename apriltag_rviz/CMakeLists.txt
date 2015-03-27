cmake_minimum_required(VERSION 2.8.3)
project(apriltag_rviz)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -Wall")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
find_package(Ogre REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    rviz apriltag_msgs
    )

message(STATUS ${${PROJECT_NAME}_HEADERS_MOC})

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Ogre_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR}
    )

qt4_wrap_cpp(${PROJECT_NAME}_HEADERS_MOC
    include/apriltag_rviz/apriltag_array_display.h
    )

add_definitions(-DQT_NO_KEYWORDS ${QT_DEFINITIONS})

catkin_package()

add_library(${PROJECT_NAME}
    src/apriltag_array_display.cpp
    src/apriltag_visual.cpp
    ${${PROJECT_NAME}_HEADERS_MOC}
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Ogre_LIBRARIES}
    ${QT_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
