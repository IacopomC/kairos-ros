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
#pragma once

#include <config_utilities/factory.h>
#include <visualization_msgs/MarkerArray.h>

#include "hydra_visualizer/plugins/visualizer_plugin.h"
#include "hydra_visualizer/utils/marker_tracker.h"

namespace kairos {
using namespace hydra;  // base-Hydra + hydra_ros/visualizer types

class FlowPlaceArrowPlugin : public VisualizerPlugin {
 public:
  struct Config {
    std::vector<int> layers{3};
    std::string occupancy_key = "flow_occupancy";
    std::string confidence_key = "flow_confidence";
    std::string components_key = "flow_components";
    std::string theta_key = "theta";
    std::string rho_key = "rho";
    std::string weight_key = "weight";
    std::string num_components_key = "flow_num_components";
    double color_r = 1.0;
    double color_g = 0.0;
    double color_b = 0.0;
    double length_scale = 0.8;
    double min_arrow_length = 0.08;
    double max_arrow_length = 1.2;
    double shaft_base_diameter = 0.02;
    double shaft_scale = 0.05;
    double head_base_diameter = 0.04;
    double head_scale = 0.10;
    double head_base_length = 0.05;
    double head_scale_length = 0.16;
    double min_alpha = 0.25;
    // When true, all arrows are drawn at z = `uniform_z` regardless of the
    // owning node's height. Gives the "floating plane above mesh_places"
    // look. When false, falls back to the original behaviour (each arrow
    // at its own node's z).
    bool use_uniform_z = true;
    double uniform_z = 0.5;
  } const config;

  FlowPlaceArrowPlugin(const Config& config,
                       const ros::NodeHandle& nh,
                       const std::string& name);

  ~FlowPlaceArrowPlugin() override = default;

  void draw(const std_msgs::Header& header,
            const spark_dsg::DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header) override;

 private:
  ros::Publisher pub_;
  mutable MarkerTracker tracker_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<VisualizerPlugin,
                                     FlowPlaceArrowPlugin,
                                     FlowPlaceArrowPlugin::Config,
                                     ros::NodeHandle,
                                     std::string>("FlowPlaceArrowPlugin");
};

void declare_config(FlowPlaceArrowPlugin::Config& config);

}  // namespace kairos
