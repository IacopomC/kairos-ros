#pragma once

#include <hydra/active_window/active_window_module.h>
#include <hydra/backend/backend_module.h>
#include <hydra/common/module.h>
#include <kairos/flow/flow_map.h>
#include <hydra/reconstruction/volumetric_map.h>
#include <kairos/temporal/flow_temporal_module.h>
#include <kairos_msgs/QueryFlow.h>
#include <odom_array_msgs/OdometryArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "kairos_ros/utils/tf_lookup.h"

namespace kairos {
using namespace hydra;  // base-Hydra + hydra_ros/visualizer types

// Thin ROS adapter around the non-ROS `FlowTemporalModule` core. Owns the
// ROS subscribers / publishers / service / TF lookup and forwards decoded
// detection frames into the core via `pushDetectionFrame`.
class FlowTemporalRosBridge : public Module {
 public:
  struct Config : public FlowTemporalModule::Config {
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

  FlowTemporalRosBridge(const Config& config, const ros::NodeHandle& nh);
  ~FlowTemporalRosBridge() override;

  void start() override;
  void stop() override;
  void save(const LogSetup& setup) override;
  std::string printInfo() const override;

  // Forward map / backend updates to the core. Same signatures as the
  // historical `FlowTemporalModule::handleMapUpdate / handleBackendUpdate`,
  // so existing `ActiveWindowModule::Sink::fromMethod` / `BackendModule::
  // Sink::fromMethod` wiring at the call site keeps working.
  void handleMapUpdate(uint64_t timestamp_ns,
                       const VolumetricMap& map,
                       const ActiveWindowOutput& msg);

  void handleBackendUpdate(uint64_t timestamp_ns,
                           const DynamicSceneGraph& graph,
                           const kimera_pgmo::DeformationGraph& dgraph);

 private:
  void detectionsCallback(const odom_array_msgs::OdometryArray::ConstPtr& msg);
  bool queryFlowCallback(kairos_msgs::QueryFlow::Request& req,
                         kairos_msgs::QueryFlow::Response& res);
  void publishVoxelMarkers(double timestamp_s);
  void publishMergeEvents(double timestamp_s);
  void publishMetrics(double timestamp_s);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber detections_sub_;
  ros::Publisher metrics_pub_;
  ros::Publisher voxel_markers_pub_;
  ros::Publisher loop_closure_merges_pub_;
  ros::ServiceServer query_flow_srv_;
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
