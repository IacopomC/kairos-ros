#include "kairos_ros/flow_temporal_ros_bridge.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <kairos/flow/flow_map.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <string>
#include <optional>
#include <sstream>

namespace kairos {
using namespace hydra;  // base-Hydra + hydra_ros/visualizer types

namespace {

double toTimestampSeconds(const ros::Time& stamp) {
  if (stamp.isZero()) {
    return ros::Time::now().toSec();
  }
  return stamp.toSec();
}

// Track id of a detection: the trailing integer of its child_frame_id
// ("12", "person_12", "track/12"). A detection without one is fed as its own
// crossing (-1).
int64_t parseTrackId(const std::string& child_frame_id) {
  size_t end = child_frame_id.size();
  size_t begin = end;
  while (begin > 0 && std::isdigit(static_cast<unsigned char>(child_frame_id[begin - 1]))) {
    --begin;
  }
  if (begin == end || end - begin > 18) {
    return -1;
  }
  return std::stoll(child_frame_id.substr(begin, end - begin));
}

}  // namespace

void declare_config(FlowTemporalRosBridge::Config& config) {
  using namespace config;
  name("FlowTemporalRosBridge::Config");
  base<FlowTemporalModule::Config>(config);
  field(config.ns, "ns");
  field(config.world_frame, "world_frame");
  field(config.observations_topic, "observations_topic");
  field(config.metrics_topic, "metrics_topic");
  field(config.voxel_markers_topic, "voxel_markers_topic");
  field(config.loop_closure_merges_topic, "loop_closure_merges_topic");
  field(config.loop_closure_merge_marker_lifetime_s,
        "loop_closure_merge_marker_lifetime_s");
  field(config.use_detection_stamp_for_tf, "use_detection_stamp_for_tf");
  field(config.tf_lookup, "tf_lookup");
}

FlowTemporalRosBridge::FlowTemporalRosBridge(const Config& config,
                                             const ros::NodeHandle& nh)
    : config(config::checkValid(config)),
      nh_(nh),
      pnh_(nh_, config.ns),
      tf_lookup_(config.tf_lookup),
      core_(static_cast<const FlowTemporalModule::Config&>(config)) {}

FlowTemporalRosBridge::~FlowTemporalRosBridge() { stop(); }

void FlowTemporalRosBridge::start() {
  if (started_) {
    return;
  }
  started_ = true;

  detections_sub_ = pnh_.subscribe(
      config.observations_topic, 1, &FlowTemporalRosBridge::detectionsCallback, this);

  if (config.publish_metrics) {
    metrics_pub_ = pnh_.advertise<std_msgs::String>(config.metrics_topic, 1, false);
  }

  if (config.publish_voxel_markers) {
    voxel_markers_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>(
        config.voxel_markers_topic, 1, true);
  }

  loop_closure_merges_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>(
      config.loop_closure_merges_topic, 10, false);

  query_flow_srv_ = pnh_.advertiseService(
      "query_flow", &FlowTemporalRosBridge::queryFlowCallback, this);

  core_.setTickCallback([this](double timestamp_s) {
    publishVoxelMarkers(timestamp_s);
    publishMergeEvents(timestamp_s);
    publishMetrics(timestamp_s);
  });

  core_.start();
}

void FlowTemporalRosBridge::stop() {
  if (!started_) {
    return;
  }

  core_.stop();

  detections_sub_.shutdown();
  metrics_pub_.shutdown();
  voxel_markers_pub_.shutdown();
  loop_closure_merges_pub_.shutdown();
  started_ = false;
}

void FlowTemporalRosBridge::save(const LogSetup& setup) { core_.save(setup); }

std::string FlowTemporalRosBridge::printInfo() const {
  return config::toString(config);
}

void FlowTemporalRosBridge::handleMapUpdate(uint64_t timestamp_ns,
                                            const VolumetricMap& map,
                                            const ActiveWindowOutput& msg) {
  core_.handleMapUpdate(timestamp_ns, map, msg);
}

void FlowTemporalRosBridge::handleBackendUpdate(
    uint64_t timestamp_ns,
    const DynamicSceneGraph& graph,
    const kimera_pgmo::DeformationGraph& dgraph) {
  core_.handleBackendUpdate(timestamp_ns, graph, dgraph);
}

void FlowTemporalRosBridge::detectionsCallback(
    const odom_array_msgs::OdometryArray::ConstPtr& msg) {
  if (!msg) {
    return;
  }

  Eigen::Isometry3d world_T_source = Eigen::Isometry3d::Identity();
  if (!msg->header.frame_id.empty() && msg->header.frame_id != config.world_frame) {
    const std::optional<ros::Time> stamp =
        config.use_detection_stamp_for_tf ? std::optional<ros::Time>(msg->header.stamp)
                                          : std::nullopt;
    const std::optional<size_t> max_tries =
        config.tf_lookup.max_tries > 0 ? std::optional<size_t>(config.tf_lookup.max_tries)
                                       : std::nullopt;

    const auto world_T_msg = lookupTransform(tf_lookup_.buffer,
                                             stamp,
                                             config.world_frame,
                                             msg->header.frame_id,
                                             max_tries,
                                             config.tf_lookup.wait_duration_s,
                                             config.tf_lookup.verbosity);
    if (!world_T_msg) {
      return;
    }

    world_T_source = world_T_msg.target_T_source();
  }

  std::vector<Eigen::Vector3d> xyz_world;
  std::vector<double> theta;
  std::vector<double> rho;
  std::vector<int64_t> track_ids;
  xyz_world.reserve(msg->odometries.size());
  theta.reserve(msg->odometries.size());
  rho.reserve(msg->odometries.size());
  track_ids.reserve(msg->odometries.size());

  for (const auto& odom : msg->odometries) {
    const Eigen::Vector3d p_source(odom.pose.pose.position.x,
                                   odom.pose.pose.position.y,
                                   odom.pose.pose.position.z);
    // Position and velocity are both expressed in the message frame; rotate the
    // velocity into the world frame so heading and position share one frame.
    const Eigen::Vector3d v_source(odom.twist.twist.linear.x,
                                   odom.twist.twist.linear.y,
                                   odom.twist.twist.linear.z);
    const Eigen::Vector3d v_world = world_T_source.linear() * v_source;
    const double speed = std::hypot(v_world.x(), v_world.y());
    if (speed <= 0.0) {
      continue;
    }

    xyz_world.push_back(world_T_source * p_source);
    theta.push_back(std::atan2(v_world.y(), v_world.x()));
    rho.push_back(speed);
    track_ids.push_back(parseTrackId(odom.child_frame_id));
  }

  const double timestamp_s = toTimestampSeconds(msg->header.stamp);
  core_.pushDetections(timestamp_s, xyz_world, theta, rho, track_ids);
}

bool FlowTemporalRosBridge::queryFlowCallback(kairos_msgs::QueryFlow::Request& req,
                                              kairos_msgs::QueryFlow::Response& res) {
  const size_t n = req.x.size();
  if (req.y.size() != n || req.z.size() != n) {
    ROS_WARN("QueryFlow: x/y/z arrays must have the same length");
    return false;
  }

  res.dominant_theta.resize(n);
  res.dominant_rho.resize(n);
  res.probability.resize(n);
  res.flags.resize(n);
  res.num_components.resize(n);

  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3f pos(static_cast<float>(req.x[i]),
                              static_cast<float>(req.y[i]),
                              static_cast<float>(req.z[i]));
    const auto result = core_.queryFlowAtPosition(pos, req.query_time_s, req.horizon_s);
    if (!result.has_value()) {
      ROS_WARN_THROTTLE(5.0, "QueryFlow: voxel grid not yet initialized");
      return false;
    }

    res.dominant_theta[i] = result->dominant_theta;
    res.dominant_rho[i] = result->dominant_rho;
    res.probability[i] = result->probability;
    res.flags[i] = result->flag;
    res.num_components[i] = result->num_components;
  }

  return true;
}

void FlowTemporalRosBridge::publishVoxelMarkers(double timestamp_s) {
  if (!config.publish_voxel_markers || !voxel_markers_pub_) {
    return;
  }

  if (voxel_markers_pub_.getNumSubscribers() == 0) {
    return;
  }

  const float voxel_size = core_.voxelSize();
  if (voxel_size <= 0.0f) {
    return;
  }

  const double query_time_s = timestamp_s;
  const auto samples = core_.flowMap().cellFlagSamples(query_time_s,
                                                       config.prediction_horizon_s,
                                                       config.voxel_markers_stride,
                                                       config.voxel_markers_max_cells);

  visualization_msgs::Marker observed;
  observed.header.frame_id = config.world_frame;
  observed.header.stamp = ros::Time::now();
  observed.ns = "flow_observed_voxels";
  observed.id = 0;
  observed.type = visualization_msgs::Marker::CUBE_LIST;
  observed.action = visualization_msgs::Marker::ADD;
  observed.pose.orientation.w = 1.0;
  observed.scale.x = voxel_size;
  observed.scale.y = voxel_size;
  observed.scale.z = voxel_size;
  observed.color.r = 0.2f;
  observed.color.g = 0.6f;
  observed.color.b = 1.0f;
  observed.color.a = 0.3f;

  visualization_msgs::Marker model_valid = observed;
  model_valid.ns = "flow_nudft_valid_voxels";
  model_valid.id = 1;
  model_valid.color.r = 1.0f;
  model_valid.color.g = 0.2f;
  model_valid.color.b = 0.2f;
  model_valid.color.a = 0.7f;

  for (const auto& sample : samples) {
    geometry_msgs::Point p;
    p.x = (static_cast<double>(sample.index.x()) + 0.5) * static_cast<double>(voxel_size);
    p.y = (static_cast<double>(sample.index.y()) + 0.5) * static_cast<double>(voxel_size);
    p.z = (static_cast<double>(sample.index.z()) + 0.5) * static_cast<double>(voxel_size);

    observed.points.push_back(p);

    if (sample.flag == CellFlag::MODEL_BASED) {
      model_valid.points.push_back(p);
    }
  }

  visualization_msgs::MarkerArray out;
  out.markers.push_back(std::move(observed));
  out.markers.push_back(std::move(model_valid));
  voxel_markers_pub_.publish(out);
}

void FlowTemporalRosBridge::publishMergeEvents(double timestamp_s) {
  // Always drain so the queue doesn't grow unbounded, even without subscribers.
  auto events = core_.flowMap().drainMergeEvents();
  total_merges_ += events.size();
  last_remap_merge_count_ = events.size();
  last_total_observations_pre_remap_ = core_.flowMap().totalObservationsPreRemap();
  last_total_observations_post_remap_ = core_.flowMap().totalObservationsPostRemap();

  // Keep a small ring of recent events for the metrics JSON.
  for (auto& event : events) {
    recent_merge_events_.push_back(event);
  }
  constexpr size_t kMaxRecent = 32;
  if (recent_merge_events_.size() > kMaxRecent) {
    recent_merge_events_.erase(
        recent_merge_events_.begin(),
        recent_merge_events_.begin() +
            static_cast<std::ptrdiff_t>(recent_merge_events_.size() - kMaxRecent));
  }

  if (events.empty() || !loop_closure_merges_pub_ ||
      loop_closure_merges_pub_.getNumSubscribers() == 0) {
    return;
  }

  const float voxel_size = core_.voxelSize();
  if (voxel_size <= 0.0f) {
    return;
  }
  const double vs = static_cast<double>(voxel_size);
  const ros::Duration lifetime(config.loop_closure_merge_marker_lifetime_s);

  visualization_msgs::Marker sources;
  sources.header.frame_id = config.world_frame;
  sources.header.stamp = ros::Time::now();
  sources.ns = "loop_closure_merge_sources";
  sources.id = 0;
  sources.type = visualization_msgs::Marker::CUBE_LIST;
  sources.action = visualization_msgs::Marker::ADD;
  sources.pose.orientation.w = 1.0;
  sources.scale.x = sources.scale.y = sources.scale.z = vs;
  sources.color.r = 1.0f;
  sources.color.g = 0.2f;
  sources.color.b = 0.2f;
  sources.color.a = 0.7f;
  sources.lifetime = lifetime;

  visualization_msgs::Marker targets = sources;
  targets.ns = "loop_closure_merge_targets";
  targets.id = 1;
  targets.color.r = 0.2f;
  targets.color.g = 1.0f;
  targets.color.b = 0.3f;

  visualization_msgs::Marker links = sources;
  links.ns = "loop_closure_merge_links";
  links.id = 2;
  links.type = visualization_msgs::Marker::LINE_LIST;
  links.scale.x = vs * 0.1;
  links.scale.y = 0.0;
  links.scale.z = 0.0;
  links.color.r = 1.0f;
  links.color.g = 0.8f;
  links.color.b = 0.0f;
  links.color.a = 0.9f;

  const auto index_to_point = [vs](const GlobalIndex& idx) {
    geometry_msgs::Point p;
    p.x = (static_cast<double>(idx.x()) + 0.5) * vs;
    p.y = (static_cast<double>(idx.y()) + 0.5) * vs;
    p.z = (static_cast<double>(idx.z()) + 0.5) * vs;
    return p;
  };

  for (const auto& event : events) {
    const auto pa = index_to_point(event.source_a);
    const auto pb = index_to_point(event.source_b);
    const auto pt = index_to_point(event.target);
    sources.points.push_back(pa);
    sources.points.push_back(pb);
    targets.points.push_back(pt);
    links.points.push_back(pa);
    links.points.push_back(pt);
    links.points.push_back(pb);
    links.points.push_back(pt);
  }

  visualization_msgs::MarkerArray out;
  out.markers.push_back(std::move(sources));
  out.markers.push_back(std::move(targets));
  out.markers.push_back(std::move(links));
  loop_closure_merges_pub_.publish(out);
  (void)timestamp_s;
}

void FlowTemporalRosBridge::publishMetrics(double timestamp_s) {
  if (!metrics_pub_) {
    return;
  }

  const auto& flow_map = core_.flowMap();
  const auto flags = flow_map.countFlags(timestamp_s, config.prediction_horizon_s);
  const auto place_summaries = core_.placeSummariesCopy();
  size_t component_count = 0;
  for (const auto& [_, summary] : place_summaries) {
    component_count += summary.components.size();
  }

  const size_t last_frame_obs = core_.lastFrameObservations();
  const size_t last_frame_used = core_.lastFrameUsedObservations();
  const size_t place_nodes_seen = core_.placeNodesSeen();
  const size_t place_nodes_with_support = core_.placeNodesWithSupport();
  const size_t place_nodes_with_components = core_.placeNodesWithComponents();
  const size_t place_summaries_places_count = core_.placeSummariesPlacesCount();

  std_msgs::String metrics;
  if (config.debug_metrics_nodes_only) {
    nlohmann::json out;
    out["timestamp_s"] = timestamp_s;
    out["method"] = "FixedComponents";
    out["cells"] = flow_map.numCells();
    out["fallback_cells"] = flags[static_cast<size_t>(CellFlag::FALLBACK)];
    out["cold_start_cells"] = flags[static_cast<size_t>(CellFlag::COLD_START)];
    out["model_based_cells"] = flags[static_cast<size_t>(CellFlag::MODEL_BASED)];
    out["total_merges"] = total_merges_;
    out["last_remap_merge_count"] = last_remap_merge_count_;
    out["total_observations_pre_remap"] = last_total_observations_pre_remap_;
    out["total_observations_post_remap"] = last_total_observations_post_remap_;
    out["frame_obs"] = last_frame_obs;
    out["frame_used"] = last_frame_used;
    out["place_summaries"] = place_summaries.size();
    out["place_summaries_places"] = place_summaries_places_count;
    out["place_nodes_seen"] = place_nodes_seen;
    out["place_nodes_with_support"] = place_nodes_with_support;
    out["place_nodes_with_components"] = place_nodes_with_components;
    out["components"] = component_count;
    out["nodes"] = nlohmann::json::array();

    int emitted = 0;
    for (const auto& [node_id, summary] : place_summaries) {
      if (emitted >= config.debug_metrics_max_nodes) {
        break;
      }

      nlohmann::json node_json;
      node_json["id"] = node_id;
      node_json["occupancy"] = summary.occupancy;
      node_json["p"] = summary.presence(config.prediction_horizon_s);
      node_json["c"] = summary.confidence;
      node_json["support"] = summary.support;
      node_json["dominant_theta"] = summary.dominant_theta;
      node_json["dominant_rho"] = summary.dominant_rho;
      node_json["num_components"] = summary.components.size();
      node_json["components"] = nlohmann::json::array();
      for (const auto& component : summary.components) {
        node_json["components"].push_back({{"theta", component.theta},
                                           {"sigma_theta", component.sigma_theta},
                                           {"sigma_rho", component.sigma_rho},
                                           {"sigma_cross", component.sigma_cross},
                                           {"rho", component.rho},
                                           {"weight", component.weight}});
      }

      out["nodes"].push_back(node_json);
      ++emitted;
    }

    nlohmann::json events_json = nlohmann::json::array();
    for (const auto& event : recent_merge_events_) {
      events_json.push_back({
          {"source_a", {event.source_a.x(), event.source_a.y(), event.source_a.z()}},
          {"source_b", {event.source_b.x(), event.source_b.y(), event.source_b.z()}},
          {"target", {event.target.x(), event.target.y(), event.target.z()}},
          {"n_a", event.n_a},
          {"n_b", event.n_b},
          {"timestamp", event.timestamp},
      });
    }
    out["recent_merge_events"] = events_json;

    metrics.data = out.dump();
    metrics_pub_.publish(metrics);
    return;
  }

  std::ostringstream ss;
  ss << "cells=" << flow_map.numCells() << " frame_obs=" << last_frame_obs
     << " frame_used=" << last_frame_used
     << " place_summaries=" << place_summaries.size()
     << " place_summaries_places=" << place_summaries_places_count
     << " place_nodes_seen=" << place_nodes_seen
     << " place_nodes_with_support=" << place_nodes_with_support
     << " place_nodes_with_components=" << place_nodes_with_components
     << " components=" << component_count
     << " fallback_cells=" << flags[static_cast<size_t>(CellFlag::FALLBACK)]
     << " cold_start_cells=" << flags[static_cast<size_t>(CellFlag::COLD_START)]
     << " model_based_cells=" << flags[static_cast<size_t>(CellFlag::MODEL_BASED)]
     << " total_merges=" << total_merges_
     << " last_remap_merge_count=" << last_remap_merge_count_
     << " total_observations_pre_remap=" << last_total_observations_pre_remap_
     << " total_observations_post_remap=" << last_total_observations_post_remap_
     << " method=fixed"
     << " timestamp_s=" << timestamp_s;
  metrics.data = ss.str();
  metrics_pub_.publish(metrics);
}

}  // namespace kairos
