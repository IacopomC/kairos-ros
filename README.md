# kairos_ros

ROS 2 (Jazzy) adapter for [KAIROS](https://github.com/IacopomC/kairos).

It provides:
- **`kairos::FlowTemporalRosBridge`** — drives `kairos::FlowTemporalModule` from
  ROS: decodes `OdometryArray` detections, publishes metrics + voxel markers,
  and serves the `QueryFlow` service. Hydra-ROS loads it as a module from its
  config.
- **Visualizer plugins** (`kairos_ros_viz`, optional; needs `hydra_visualizer`):
  `FlowPlaceArrowPlugin` renders per-place flow arrows, and
  `TemporalPlaceColorAdapter` colors places by how busy the flow model expects
  them to be.
- **`flow_voxel_publisher.py`** — publishes the flow voxels of a saved run.
- **`kairos.launch.yaml`** — Hydra-ROS with the flow module.

Each detection is one `nav_msgs/Odometry` in the `OdometryArray`: position in
`pose`, velocity in `twist`, both in the message's `header.frame_id`, and the
track id as the trailing integer of `child_frame_id` (e.g. `person_12`).

The `QueryFlow` service lives in the separate
[`kairos_msgs`](https://github.com/IacopomC/kairos-msgs) package.

`kairos_ros` builds on `kairos`, `kairos_msgs`, `hydra_ros` and `rclcpp`.
Hydra-ROS itself has no dependency on KAIROS.

---

## Clone & build

`kairos_ros` is built as part of the KAIROS colcon workspace (see the
[`kairos` README](https://github.com/IacopomC/kairos#installation)):

```bash
cd kairos_ws
colcon build --packages-up-to kairos_ros
colcon build --packages-up-to kairos_ros --cmake-args -DKAIROS_ROS_BUILD_PLUGIN=OFF  # without the visualizer plugins
```

---

## Running with ROS

```bash
ros2 launch kairos_ros kairos.launch.yaml dataset:=tbd input:=unizar \
    labelspace:=ade20k_outdoor robot_frame:=base_link
```

`dataset` names a KAIROS dataset config (`kairos/config/datasets`), which holds
the flow parameters; `input` names a ROS input config
(`kairos_ros/config/datasets`). The launch file includes Hydra-ROS's
`hydra.launch.yaml`, loads the `kairos_ros` library into it and runs the flow
module as `FlowTemporalRosBridge`.

Kairos reads the detections on `/people_detections_odom` (launch argument
`observations_topic`). The bridge's topics and service live under the Hydra
node's `~/flow_temporal` namespace:

| Name | Type |
|---|---|
| `/hydra/flow_temporal/query_flow` | `kairos_msgs/srv/QueryFlow` |
| `/hydra/flow_temporal/temporal/flow_metrics` | `std_msgs/msg/String` |
| `/hydra/flow_temporal/temporal/flow_voxel_markers` | `visualization_msgs/msg/MarkerArray` |
| `/hydra/flow_temporal/temporal/loop_closure_merges` | `visualization_msgs/msg/MarkerArray` |

---

## Visualization

`kairos.launch.yaml` starts Hydra's streaming visualizer with the Kairos
visualizer config (`config/visualizer_config.yaml`) and plugins
(`config/visualizer_plugins.yaml`); `start_visualizer:=false` turns it off.

To publish the flow voxels of a saved run on their own:

```bash
ros2 run kairos_ros flow_voxel_publisher.py --ros-args \
    -p scene_graph:=/out/run01/dsg.json \
    -p frame_id:=map -p rate_hz:=1.0 -p max_cells:=50000
```

| Param | Default | Meaning |
|---|---|---|
| `scene_graph` | (required) | saved `dsg.json` carrying `graph.metadata.flow`; without it the node warns and idles |
| `frame_id` | `map` | frame the markers are published in |
| `topic` | `/hydra_dsg_visualizer/flow_voxels` | marker topic |
| `rate_hz` | `1.0` | republish rate |
| `stride` | `1` | publish every Nth cell, to thin a dense map |
| `max_cells` | `50000` | hard cap on published cells |
| `voxel_size` | `0.1` | marker cube size, used only when the DSG carries no usable voxel size |

The DSG carries flow metadata when the run had the flow module configured
(`modules.flow_temporal` in the dataset YAML); a DSG without flow metadata
publishes no markers.

## Layout
```
kairos_ros/
├── config/
│   ├── datasets/unizar.yaml
│   ├── visualizer_config.yaml
│   └── visualizer_plugins.yaml
├── include/kairos_ros/
│   ├── flow_temporal_ros_bridge.h
│   └── plugins/
│       ├── flow_place_arrow_plugin.h
│       └── temporal_place_color_adapter.h
├── launch/kairos.launch.yaml
├── src/
│   ├── flow_temporal_ros_bridge.cpp
│   └── plugins/
│       ├── flow_place_arrow_plugin.cpp
│       └── temporal_place_color_adapter.cpp
├── scripts/flow_voxel_publisher.py
├── CMakeLists.txt
└── package.xml
```
