#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#
cmake_minimum_required(VERSION 3.5)
project(apriltag_umich LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS opencv_core)

add_subdirectory(apriltag)

add_library(${PROJECT_NAME} INTERFACE)
add_dependencies(${PROJECT_NAME} apriltag)

target_link_libraries(${PROJECT_NAME} INTERFACE opencv_core apriltag::apriltag)

target_include_directories(${PROJECT_NAME}
  INTERFACE $<INSTALL_INTERFACE:include/apriltag>
  ${OpenCV_INCLUDE_DIRS}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})

# need export_dependencies() or else you have to
# have find_package(apriltag) in your package to
# use apriltag_umich
ament_export_dependencies(apriltag)

ament_package()
