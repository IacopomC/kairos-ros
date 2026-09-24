cmake_minimum_required(VERSION 3.16)
project(kairos_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(KAIROS_ROS_BUILD_PLUGIN
       "Build the rviz flow-arrow plugin (needs hydra_visualizer)" OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  std_msgs
  visualization_msgs
  odom_array_msgs
  kairos_msgs
)
find_package(hydra REQUIRED)
find_package(kairos REQUIRED)
find_package(config_utilities REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kairos_ros
  CATKIN_DEPENDS roscpp tf2_ros std_msgs visualization_msgs odom_array_msgs kairos_msgs
)

add_library(kairos_ros
  src/flow_temporal_ros_bridge.cpp
  src/utils/tf_lookup.cpp
)
target_include_directories(kairos_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(kairos_ros PUBLIC
  kairos::kairos
  hydra::hydra
  config_utilities::config_utilities
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)
# Wait for kairos_msgs (QueryFlow.h) to be generated.
add_dependencies(kairos_ros ${catkin_EXPORTED_TARGETS})

# --- optional rviz plugin (couples to hydra_visualizer) ---
if(KAIROS_ROS_BUILD_PLUGIN)
  find_package(hydra_visualizer REQUIRED)
  add_library(kairos_ros_plugins src/plugins/flow_place_arrow_plugin.cpp)
  target_include_directories(kairos_ros_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${catkin_INCLUDE_DIRS} ${hydra_visualizer_INCLUDE_DIRS})
  target_link_libraries(kairos_ros_plugins PUBLIC
    kairos_ros ${hydra_visualizer_LIBRARIES} ${catkin_LIBRARIES})
  install(TARGETS kairos_ros_plugins
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

install(TARGETS kairos_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
catkin_install_python(PROGRAMS scripts/flow_voxel_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
