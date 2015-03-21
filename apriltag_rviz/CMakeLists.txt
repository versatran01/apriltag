cmake_minimum_required(VERSION 2.8.3)
project(apriltag_rviz)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
find_package(Ogre REQUIRED)
find_package(catkin REQUIRED COMPONENTS rviz)

include_directories(${Ogre_INCLUDE_DIRS})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
