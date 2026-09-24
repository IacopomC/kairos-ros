# kairos_ros

ROS (Noetic) adapter for [KAIROS](https://github.com/IacopomC/kairos).

It provides:
- **`kairos::FlowTemporalRosBridge`** — drives `kairos::FlowTemporalModule` from
  ROS: decodes `OdometryArray` detections, publishes metrics + voxel markers,
  and serves the `QueryFlow` service.
- **`flow_place_arrow_plugin`** — an rviz plugin that renders per-place flow
  arrows (optional; needs `hydra_visualizer`).
- **`flow_voxel_publisher.py`** — publishes flow voxels for visualization.

Each detection is one `nav_msgs/Odometry` in the `OdometryArray`: position in
`pose`, velocity in `twist`, both in the message's `header.frame_id`, and the
track id as the trailing integer of `child_frame_id` (e.g. `person_12`).

The `QueryFlow` service lives in the separate
[`kairos_msgs`](https://github.com/IacopomC/kairos-msgs) package.

`kairos_ros` builds on `kairos`, `kairos_msgs` and `roscpp`.

---

## Clone & build

`kairos_ros` is built as part of the KAIROS workspace. Pull the whole workspace
with vcstool (see the [`kairos` README](https://github.com/IacopomC/kairos#installation)):

```bash
mkdir -p kairos_ws/src && cd kairos_ws/src
git clone https://github.com/IacopomC/kairos.git
vcs import . < kairos/install/kairos.rosinstall   # kairos-ros, kairos-msgs, hydra forks, deps
```

### Native (Ubuntu 20.04 + ROS Noetic)
```bash
cd kairos_ws
catkin build kairos_ros                              # core: flow ROS bridge
catkin build kairos_ros -DKAIROS_ROS_BUILD_PLUGIN=ON # + rviz flow-arrow plugin
```

### Docker
In the toolchain image (`kairos/docker/build_image.sh --dev`), from the workspace `src/`:
```bash
kairos/docker/run.sh bash catkin_build.sh kairos_msgs kairos_ros
```

---

## Running with ROS

`kairos_ros` runs inside Hydra's ROS node (`hydra-kairos-ros`). Launch Hydra
for your sensor; the flow layer is on by default:

```bash
roslaunch hydra_ros <dataset>.launch      # e.g. uhumans2.launch
```

Kairos reads the detections on `/people_detections_odom` (launch argument
`flow_temporal_observations_topic`). Its parameters are the launch arguments of
`hydra_ros/launch/pipeline_config.launch` in `hydra-kairos-ros`.

---

## Visualization

Launch `hydra_visualizer.launch` from `hydra-kairos-ros` to view a scene graph
in rviz. The rviz flow-arrow plugin must be built with `-DKAIROS_ROS_BUILD_PLUGIN=ON`.

To publish the flow voxels of a saved run on their own:

```bash
rosrun kairos_ros flow_voxel_publisher.py \
    _scene_graph:=/out/run01/dsg.json \
    _frame_id:=map _rate_hz:=1.0 _max_cells:=50000
```

| Param | Default | Meaning |
|---|---|---|
| `~scene_graph` | (required) | saved `dsg.json` carrying `graph.metadata.flow`; without it the node warns and idles |
| `~frame_id` | `map` | frame the markers are published in |
| `~topic` | `/hydra_dsg_visualizer/flow_voxels` | marker topic |
| `~rate_hz` | `1.0` | republish rate |
| `~stride` | `1` | publish every Nth cell, to thin a dense map |
| `~max_cells` | `50000` | hard cap on published cells |
| `~voxel_size` | `0.1` | marker cube size, used only when the DSG carries no usable voxel size |

The DSG carries flow metadata when the run had the temporal module enabled
(`enable_flow_temporal_module: true` in the dataset YAML); a DSG without flow
metadata publishes no markers.

## Layout
```
kairos_ros/
├── include/kairos_ros/
│   ├── flow_temporal_ros_bridge.h
│   ├── utils/tf_lookup.h
│   └── plugins/flow_place_arrow_plugin.h
├── src/
│   ├── flow_temporal_ros_bridge.cpp
│   ├── utils/tf_lookup.cpp
│   └── plugins/flow_place_arrow_plugin.cpp
├── scripts/flow_voxel_publisher.py
├── CMakeLists.txt
└── package.xml
```
