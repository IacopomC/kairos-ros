#pragma once

#include <hydra/common/module.h>
#include <hydra_ros/utils/tf_lookup.h>
#include <ianvs/node_handle.h>
#include <kairos/flow/flow_map.h>
#include <kairos/temporal/flow_temporal_module.h>

#include <cstdint>
#include <kairos_msgs/srv/query_flow.hpp>
#include <memory>
#include <odom_array_msgs/msg/odometry_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace kairos {
using namespace hydra;  // base-Hydra + hydra_ros/visualizer types

// Thin ROS adapter around the non-ROS `FlowTemporalModule` core. Owns the
// ROS subscribers / publishers / service / TF lookup and forwards decoded
// detection frames into the core via `pushDetectionFrame`. Created by Hydra-ROS
// from its `modules` config ("FlowTemporalRosBridge"); map and backend updates
// reach the core through FlowMapSink and FlowUpdateFunctor, which find it by
// name.
class FlowTemporalRosBridge : public Module {
 public:
  struct Config : public FlowTemporalModule::Config {
    //! Namespace of the topics and service, under the node's ~/flow_temporal
    std::string ns = "";
    std::string world_frame = "map";
    std::string observations_topic = "temporal/odometry_array";
    std::string metrics_topic = "temporal/flow_metrics";
    std::string voxel_markers_topic = "temporal/flow_voxel_markers";
    std::string loop_closure_merges_topic = "temporal/loop_closure_merges";
    double loop_closure_merge_marker_lifetime_s = 3.0;
    bool use_detection_stamp_for_tf = true;
    TFLookup::Config tf_lookup;
  } const config;

  explicit FlowTemporalRosBridge(const Config& config);
  ~FlowTemporalRosBridge() override;

  void start() override;
  void stop() override;
  void save(const DataDirectory& output) override;
  std::string printInfo() const override;

 private:
  using OdometryArray = odom_array_msgs::msg::OdometryArray;
  using QueryFlow = kairos_msgs::srv::QueryFlow;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  void detectionsCallback(const OdometryArray::ConstSharedPtr& msg);
  void queryFlowCallback(const QueryFlow::Request::SharedPtr req,
                         QueryFlow::Response::SharedPtr res);
  void publishVoxelMarkers(double timestamp_s);
  void publishMergeEvents(double timestamp_s);
  void publishMetrics(double timestamp_s);

  ianvs::NodeHandle nh_;
  rclcpp::Subscription<OdometryArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr voxel_markers_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr loop_closure_merges_pub_;
  rclcpp::Service<QueryFlow>::SharedPtr query_flow_srv_;
  size_t total_merges_ = 0;
  size_t last_remap_merge_count_ = 0;
  int64_t last_total_observations_pre_remap_ = 0;
  int64_t last_total_observations_post_remap_ = 0;
  std::vector<MergeEvent> recent_merge_events_;

  TFLookup tf_lookup_;
  FlowTemporalModule core_;
  bool started_ = false;
};

void declare_config(FlowTemporalRosBridge::Config& config);

}  // namespace kairos
