#pragma once

#include <config_utilities/virtual_config.h>
#include <hydra_visualizer/adapters/graph_color.h>
#include <spark_dsg/color.h>

#include <string>

namespace kairos {

/**
 * @brief Colors places by how busy the flow model expects them to be.
 *
 * Green to red by the presence a place's occupancy induces right now
 * (1 - exp(-occupancy)), with the alpha following its flow confidence. Nodes
 * without flow metadata use the fallback adapter.
 */
struct TemporalPlaceColorAdapter : public hydra::GraphColorAdapter {
  struct Config {
    config::VirtualConfig<hydra::GraphColorAdapter> fallback_adapter;
    spark_dsg::Color low_color{0, 255, 0};
    spark_dsg::Color high_color{255, 0, 0};
    /// Metadata key holding the place's occupancy, the mean number of targets
    /// present. Places store an occupancy rather than a probability so that no
    /// stored attribute is tied to a prediction horizon; the adapter maps it
    /// onto the colour ramp through the instantaneous presence it induces.
    std::string occupancy_key = "flow_occupancy";
    bool use_confidence_alpha = true;
    std::string confidence_key = "flow_confidence";
    float min_alpha = 0.25f;
    float max_alpha = 1.0f;
  } const config;

  explicit TemporalPlaceColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;
  void setGraph(const spark_dsg::DynamicSceneGraph& graph,
                spark_dsg::LayerId layer) override;

 private:
  hydra::GraphColorAdapter::Ptr fallback_;
};

void declare_config(TemporalPlaceColorAdapter::Config& config);

}  // namespace kairos
