#include "kairos_ros/plugins/temporal_place_color_adapter.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_node.h>

#include <algorithm>
#include <cmath>

namespace kairos {

using spark_dsg::Color;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::LayerId;
using spark_dsg::SceneGraphNode;

namespace {

const auto registration =
    config::RegistrationWithConfig<hydra::GraphColorAdapter,
                                   TemporalPlaceColorAdapter,
                                   TemporalPlaceColorAdapter::Config>(
        "TemporalPlaceColorAdapter");

}  // namespace

TemporalPlaceColorAdapter::TemporalPlaceColorAdapter(const Config& config)
    : config(config::checkValid(config)),
      fallback_(config.fallback_adapter ? config.fallback_adapter.create() : nullptr) {}

Color TemporalPlaceColorAdapter::getColor(const DynamicSceneGraph& graph,
                                          const SceneGraphNode& node) const {
  const auto& metadata = node.attributes().metadata();
  if (!metadata.contains(config.occupancy_key)) {
    return fallback_ ? fallback_->getColor(graph, node) : Color();
  }

  const auto value = metadata.at(config.occupancy_key);
  if (!value.is_number()) {
    return fallback_ ? fallback_->getColor(graph, node) : Color();
  }

  // Colour by the presence the occupancy induces right now, which lands in
  // [0, 1] by construction and keeps busier places redder.
  const double occupancy = std::max(0.0, value.get<double>());
  const float p = std::clamp(static_cast<float>(1.0 - std::exp(-occupancy)), 0.0f, 1.0f);
  Color color;
  color.r = static_cast<uint8_t>((1.0f - p) * config.low_color.r + p * config.high_color.r);
  color.g = static_cast<uint8_t>((1.0f - p) * config.low_color.g + p * config.high_color.g);
  color.b = static_cast<uint8_t>((1.0f - p) * config.low_color.b + p * config.high_color.b);

  float alpha = config.max_alpha;
  if (config.use_confidence_alpha && metadata.contains(config.confidence_key)) {
    const auto confidence = metadata.at(config.confidence_key);
    if (confidence.is_number()) {
      const float c = std::clamp(static_cast<float>(confidence.get<double>()), 0.0f, 1.0f);
      alpha = config.min_alpha + (config.max_alpha - config.min_alpha) * c;
    }
  }

  alpha = std::clamp(alpha, 0.0f, 1.0f);
  color.a = static_cast<uint8_t>(alpha * 255.0f);
  return color;
}

void TemporalPlaceColorAdapter::setGraph(const DynamicSceneGraph& graph, LayerId layer) {
  if (fallback_) {
    fallback_->setGraph(graph, layer);
  }
}

void declare_config(TemporalPlaceColorAdapter::Config& config) {
  using namespace config;
  name("TemporalPlaceColorAdapter::Config");
  config.fallback_adapter.setOptional();
  field(config.fallback_adapter, "fallback_adapter");
  field(config.low_color, "low_color");
  field(config.high_color, "high_color");
  field(config.occupancy_key, "occupancy_key");
  field(config.use_confidence_alpha, "use_confidence_alpha");
  field(config.confidence_key, "confidence_key");
  field(config.min_alpha, "min_alpha");
  check(config.min_alpha, GE, 0.0f, "min_alpha");
  check(config.min_alpha, LE, 1.0f, "min_alpha");
  field(config.max_alpha, "max_alpha");
  check(config.max_alpha, GE, 0.0f, "max_alpha");
  check(config.max_alpha, LE, 1.0f, "max_alpha");
  check(config.max_alpha, GE, config.min_alpha, "max_alpha_ge_min_alpha");
}

}  // namespace kairos
