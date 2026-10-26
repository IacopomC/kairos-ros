/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */

#include "kairos_ros/plugins/flow_place_arrow_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>

#include <algorithm>
#include <cmath>

namespace kairos {

namespace {

const auto registration =
    config::RegistrationWithConfig<VisualizerPlugin,
                                   FlowPlaceArrowPlugin,
                                   FlowPlaceArrowPlugin::Config,
                                   ianvs::NodeHandle,
                                   std::string>("FlowPlaceArrowPlugin");

}  // namespace
using namespace hydra;  // base-Hydra + hydra_ros/visualizer types

using spark_dsg::DynamicSceneGraph;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

double readDouble(const nlohmann::json& metadata,
                  const std::string& key,
                  double fallback = 0.0) {
  if (!metadata.is_object()) {
    return fallback;
  }

  const auto iter = metadata.find(key);
  if (iter == metadata.end()) {
    return fallback;
  }

  if (iter->is_number_float() || iter->is_number_integer() || iter->is_number_unsigned()) {
    return iter->get<double>();
  }

  return fallback;
}

bool readBool(const nlohmann::json& metadata,
              const std::string& key,
              bool fallback = false) {
  if (!metadata.is_object()) {
    return fallback;
  }

  const auto iter = metadata.find(key);
  if (iter == metadata.end() || !iter->is_boolean()) {
    return fallback;
  }

  return iter->get<bool>();
}

bool hasFlowSupport(const nlohmann::json& metadata,
                    const std::string& num_components_key) {
  return readDouble(metadata, num_components_key, 0.0) > 0.0;
}

double clamp01(double value) {
  return std::clamp(value, 0.0, 1.0);
}

bool parseComponent(const nlohmann::json& component,
                    const std::string& theta_key,
                    const std::string& rho_key,
                    const std::string& weight_key,
                    double& theta,
                    double& rho,
                    double& weight) {
  if (!component.is_object()) {
    return false;
  }

  if (!component.contains(theta_key) || !component.contains(rho_key) ||
      !component.contains(weight_key)) {
    return false;
  }

  const auto& theta_json = component.at(theta_key);
  const auto& rho_json = component.at(rho_key);
  const auto& weight_json = component.at(weight_key);
  if (!theta_json.is_number() || !rho_json.is_number() || !weight_json.is_number()) {
    return false;
  }

  theta = theta_json.get<double>();
  rho = std::max(0.0, rho_json.get<double>());
  weight = clamp01(weight_json.get<double>());
  return true;
}

}  // namespace

void declare_config(FlowPlaceArrowPlugin::Config& config) {
  using namespace config;
  name("FlowPlaceArrowPlugin::Config");
  field(config.layers, "layers");
  field(config.occupancy_key, "occupancy_key");
  field(config.confidence_key, "confidence_key");
  field(config.components_key, "components_key");
  field(config.theta_key, "theta_key");
  field(config.rho_key, "rho_key");
  field(config.weight_key, "weight_key");
  field(config.num_components_key, "num_components_key");
  field(config.color_r, "color_r");
  field(config.color_g, "color_g");
  field(config.color_b, "color_b");
  field(config.length_scale, "length_scale");
  field(config.min_arrow_length, "min_arrow_length");
  field(config.max_arrow_length, "max_arrow_length");
  field(config.shaft_base_diameter, "shaft_base_diameter");
  field(config.shaft_scale, "shaft_scale");
  field(config.head_base_diameter, "head_base_diameter");
  field(config.head_scale, "head_scale");
  field(config.head_base_length, "head_base_length");
  field(config.head_scale_length, "head_scale_length");
  field(config.min_alpha, "min_alpha");
  field(config.use_uniform_z, "use_uniform_z");
  field(config.uniform_z, "uniform_z");

  check(config.length_scale, GT, 0.0, "length_scale");
  check(config.min_arrow_length, GE, 0.0, "min_arrow_length");
  check(config.max_arrow_length, GT, 0.0, "max_arrow_length");
  check(config.max_arrow_length, GE, config.min_arrow_length, "max_arrow_length");
  check(config.shaft_base_diameter, GT, 0.0, "shaft_base_diameter");
  check(config.shaft_scale, GE, 0.0, "shaft_scale");
  check(config.head_base_diameter, GT, 0.0, "head_base_diameter");
  check(config.head_scale, GE, 0.0, "head_scale");
  check(config.head_base_length, GT, 0.0, "head_base_length");
  check(config.head_scale_length, GE, 0.0, "head_scale_length");
  check(config.min_alpha, GE, 0.0, "min_alpha");
  check(config.min_alpha, LE, 1.0, "min_alpha");
  check(config.color_r, GE, 0.0, "color_r");
  check(config.color_r, LE, 1.0, "color_r");
  check(config.color_g, GE, 0.0, "color_g");
  check(config.color_g, LE, 1.0, "color_g");
  check(config.color_b, GE, 0.0, "color_b");
  check(config.color_b, LE, 1.0, "color_b");
}

FlowPlaceArrowPlugin::FlowPlaceArrowPlugin(const Config& config,
                                           ianvs::NodeHandle nh,
                                           const std::string& name)
    : VisualizerPlugin(name),
      config(config::checkValid(config)),
      pub_(nh.create_publisher<MarkerArray>(name, rclcpp::QoS(1).transient_local())) {}

void FlowPlaceArrowPlugin::draw(const std_msgs::msg::Header& header,
                                const DynamicSceneGraph& graph) {
  if (!pub_) {
    return;
  }

  MarkerArray msg;
  int marker_id = 0;

  for (const auto& layer_name : config.layers) {
    if (!graph.hasLayer(layer_name)) {
      continue;
    }

    const auto& layer = graph.getLayer(layer_name);
    const std::string ns = "flow_place_arrows_" + layer_name;

    for (const auto& id_node_pair : layer.nodes()) {
      const auto& attrs = id_node_pair.second->attributes();
      if (!hasFlowSupport(attrs.metadata(), config.num_components_key)) {
        continue;
      }

      const double confidence = clamp01(readDouble(attrs.metadata(), config.confidence_key, 1.0));
      const bool show_dominant_overlay = readBool(attrs.metadata(), "flow_use_fixed_components", false);
      const double base_z =
          config.use_uniform_z ? config.uniform_z : attrs.position.z();

      bool emitted_component_arrow = false;
      if (attrs.metadata().contains(config.components_key) &&
          attrs.metadata().at(config.components_key).is_array()) {
        for (const auto& component : attrs.metadata().at(config.components_key)) {
          double theta = 0.0;
          double rho = 0.0;
          double weight = 0.0;
          if (!parseComponent(component,
                              config.theta_key,
                              config.rho_key,
                              config.weight_key,
                              theta,
                              rho,
                              weight)) {
            continue;
          }

          const double arrow_length = std::clamp(config.length_scale * rho * (0.25 + 0.75 * weight),
                                                 config.min_arrow_length,
                                                 config.max_arrow_length);
          if (arrow_length <= 0.0) {
            continue;
          }

          Marker marker;
          marker.header = header;
          marker.ns = ns;
          marker.id = marker_id++;
          marker.type = Marker::ARROW;
          marker.action = Marker::ADD;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = config.shaft_base_diameter + config.shaft_scale * weight;
          marker.scale.y = config.head_base_diameter + config.head_scale * weight;
          marker.scale.z = config.head_base_length + config.head_scale_length * weight;

          geometry_msgs::msg::Point start;
          start.x = attrs.position.x();
          start.y = attrs.position.y();
          start.z = base_z;
          marker.points.push_back(start);

          geometry_msgs::msg::Point end;
          end.x = attrs.position.x() + arrow_length * std::cos(theta);
          end.y = attrs.position.y() + arrow_length * std::sin(theta);
          end.z = base_z;
          marker.points.push_back(end);

          marker.color.r = static_cast<float>(config.color_r);
          marker.color.g = static_cast<float>(config.color_g);
          marker.color.b = static_cast<float>(config.color_b);
          marker.color.a = static_cast<float>(std::max(config.min_alpha, confidence * weight));

          tracker_.add(marker, msg);
          emitted_component_arrow = true;
        }
      }

      if (!show_dominant_overlay) {
        continue;
      }

      const double theta = readDouble(attrs.metadata(), "flow_dominant_theta", 0.0);
      const double rho = std::max(0.0, readDouble(attrs.metadata(), "flow_dominant_rho", 0.0));
      const double arrow_length = std::clamp(config.length_scale * rho,
                                             config.min_arrow_length,
                                             config.max_arrow_length);
      if (arrow_length <= 0.0) {
        continue;
      }

      Marker marker;
      marker.header = header;
      marker.ns = ns;
      marker.id = marker_id++;
      marker.type = Marker::ARROW;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = config.shaft_base_diameter + config.shaft_scale;
      marker.scale.y = config.head_base_diameter + config.head_scale;
      marker.scale.z = config.head_base_length + config.head_scale_length;

      geometry_msgs::msg::Point start;
      start.x = attrs.position.x();
      start.y = attrs.position.y();
      start.z = base_z + (emitted_component_arrow ? 0.08 : 0.0);
      marker.points.push_back(start);

      geometry_msgs::msg::Point end;
      end.x = attrs.position.x() + arrow_length * std::cos(theta);
      end.y = attrs.position.y() + arrow_length * std::sin(theta);
      end.z = base_z + (emitted_component_arrow ? 0.08 : 0.0);
      marker.points.push_back(end);

      marker.color.r = static_cast<float>(config.color_r);
      marker.color.g = static_cast<float>(config.color_g);
      marker.color.b = static_cast<float>(config.color_b);
      marker.color.a = static_cast<float>(std::max(std::max(config.min_alpha, 0.55), confidence));

      if (emitted_component_arrow) {
        marker.scale.x = std::max(marker.scale.x, config.shaft_base_diameter + 1.5 * config.shaft_scale);
        marker.scale.y = std::max(marker.scale.y, config.head_base_diameter + 1.5 * config.head_scale);
        marker.scale.z = std::max(marker.scale.z,
                                  config.head_base_length + 1.5 * config.head_scale_length);
      }

      tracker_.add(marker, msg);
    }
  }

  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

void FlowPlaceArrowPlugin::reset(const std_msgs::msg::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

}  // namespace kairos
