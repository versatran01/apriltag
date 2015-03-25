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

include_directories(include
    ${Ogre_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR}
    )

add_definitions(-DQT_NO_KEYWORDS)

catkin_package()

add_executable(test_pub_pose test/test_pub_pose.cpp)
target_link_libraries(test_pub_pose ${catkin_LIBRARIES})
