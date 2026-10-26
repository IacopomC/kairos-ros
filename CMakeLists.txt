cmake_minimum_required(VERSION 3.16)
project(kairos_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(KAIROS_ROS_BUILD_PLUGIN
       "Build the visualizer flow plugins (needs hydra_visualizer)" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(odom_array_msgs REQUIRED)
find_package(kairos_msgs REQUIRED)
find_package(hydra REQUIRED)
find_package(hydra_ros REQUIRED)
find_package(ianvs REQUIRED)
find_package(kairos REQUIRED)
find_package(config_utilities REQUIRED)
find_package(Eigen3 REQUIRED)

# Loaded by hydra_ros_node through its `paths` setting; registers
# FlowTemporalRosBridge (and, through libkairos, the flow sink and functors).
add_library(kairos_ros SHARED src/flow_temporal_ros_bridge.cpp)
target_include_directories(kairos_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kairos_ros PUBLIC
  kairos::kairos
  hydra::hydra
  hydra_ros::hydra_ros
  ianvs::ianvs
  config_utilities::config_utilities
  Eigen3::Eigen
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${odom_array_msgs_TARGETS}
  ${kairos_msgs_TARGETS}
)

# --- optional visualizer plugins (couple to hydra_visualizer) ---
if(KAIROS_ROS_BUILD_PLUGIN)
  find_package(hydra_visualizer REQUIRED)
  add_library(kairos_ros_viz SHARED
    src/plugins/flow_place_arrow_plugin.cpp
    src/plugins/temporal_place_color_adapter.cpp
  )
  target_include_directories(kairos_ros_viz PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(kairos_ros_viz PUBLIC
    hydra_visualizer::hydra_visualizer
    ianvs::ianvs
    config_utilities::config_utilities
    rclcpp::rclcpp
    ${visualization_msgs_TARGETS}
  )
  install(TARGETS kairos_ros_viz
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(TARGETS kairos_ros
  EXPORT kairos_ros-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/flow_voxel_publisher.py DESTINATION lib/${PROJECT_NAME})

ament_export_targets(kairos_ros-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hydra hydra_ros kairos kairos_msgs odom_array_msgs)
ament_package()
