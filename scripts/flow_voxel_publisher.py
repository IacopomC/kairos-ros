#!/usr/bin/env python3
"""Publish CUBE_LIST markers from a saved DSG's embedded flow map.

The live FlowTemporalRosBridge publishes voxel markers directly from
the in-memory FlowMap. For offline replay (loading a saved DSG via
hydra_visualizer.launch) the in-memory state isn't available, so this
node reads the flow map embedded in the DSG JSON (top-level
`metadata.flow.map`, populated each backend update by
FlowTemporalModule) and publishes the equivalent markers on a
configurable topic.

Two markers are published:
- flow_observed_voxels: every cell in the flow map.
- flow_nudft_valid_voxels: cells whose model is MODEL_BASED
  (i.e., have a usable predictor).

Parameters are ROS parameters, not argparse:

    ros2 run kairos_ros flow_voxel_publisher.py --ros-args \
        -p scene_graph:=/out/run01/dsg.json \
        -p frame_id:=map -p rate_hz:=1.0 -p max_cells:=50000

Without `scene_graph` the node warns and idles. `frame_id` (map), `topic`
(/hydra_dsg_visualizer/flow_voxels), `rate_hz` (1.0), `stride` (1),
`max_cells` (50000) and `voxel_size` (0.1, used only when the DSG carries no
usable voxel size of its own) have defaults. A run whose temporal module was
disabled carries no flow metadata, so nothing is published.
"""

import json
import os
import sys

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def derive_flag(cell):
    """Replicate FlowCellState::predict's flag selection from JSON.

    The serialized cell carries the same fields the live state uses to
    decide its CellFlag. Mirroring the rule here keeps the offline
    visualization consistent with what RViz showed live.
    """
    if cell.get("total_observations", 0) == 0:
        return "FALLBACK"
    if not cell.get("cell_reliable", True):
        return "FALLBACK"
    if not cell.get("initialized", True):
        return "COLD_START"
    if not cell.get("nudft_valid", False):
        return "COLD_START"
    return "MODEL_BASED"


def load_flow_from_dsg(scene_graph_path, logger):
    """Read graph.metadata.flow from a saved DSG JSON.

    Returns (cells, voxel_size). cells is a list of cell records as
    serialized by FlowMap::toJson; voxel_size is in meters. Returns
    ([], None) if the DSG carries no flow metadata, which is the case
    when the run had the temporal module disabled.
    """
    with open(scene_graph_path) as f:
        dsg = json.load(f)
    flow = (dsg.get("metadata") or {}).get("flow")
    if not flow:
        return [], None
    flow_map = flow.get("map") or {}
    if flow_map.get("schema") != "hydra_flow_map_checkpoint":
        logger.error(
            f"[flow_voxel_publisher] DSG flow.map at {scene_graph_path} has unexpected schema"
        )
        return [], None
    return flow_map.get("cells", []), flow.get("voxel_size")


def build_markers(cells, voxel_size, frame_id, stride, max_cells, stamp):
    observed = Marker()
    observed.header.frame_id = frame_id
    observed.header.stamp = stamp
    observed.ns = "flow_observed_voxels"
    observed.id = 0
    observed.type = Marker.CUBE_LIST
    observed.action = Marker.ADD
    observed.pose.orientation.w = 1.0
    observed.scale.x = voxel_size
    observed.scale.y = voxel_size
    observed.scale.z = voxel_size
    observed.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.3)

    model_valid = Marker()
    model_valid.header = observed.header
    model_valid.ns = "flow_nudft_valid_voxels"
    model_valid.id = 1
    model_valid.type = Marker.CUBE_LIST
    model_valid.action = Marker.ADD
    model_valid.pose.orientation.w = 1.0
    model_valid.scale.x = voxel_size
    model_valid.scale.y = voxel_size
    model_valid.scale.z = voxel_size
    model_valid.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.7)

    count = 0
    for i, cell in enumerate(cells):
        if stride > 1 and (i % stride) != 0:
            continue
        if max_cells > 0 and count >= max_cells:
            break
        idx = cell.get("index")
        if not (isinstance(idx, list) and len(idx) == 3):
            continue
        p = Point()
        p.x = (float(idx[0]) + 0.5) * voxel_size
        p.y = (float(idx[1]) + 0.5) * voxel_size
        p.z = (float(idx[2]) + 0.5) * voxel_size
        observed.points.append(p)
        if derive_flag(cell) == "MODEL_BASED":
            model_valid.points.append(p)
        count += 1

    return MarkerArray(markers=[observed, model_valid])


class FlowVoxelPublisher(Node):
    """Publishes the flow voxels of a saved DSG."""

    def __init__(self):
        super().__init__("flow_voxel_publisher")
        self._timer = None
        scene_graph_path = self.declare_parameter("scene_graph", "").value
        if not scene_graph_path:
            self.get_logger().warning("[flow_voxel_publisher] no scene_graph path provided; idling")
            return

        if not os.path.exists(scene_graph_path):
            self.get_logger().error(
                f"[flow_voxel_publisher] scene_graph not found: {scene_graph_path}"
            )
            sys.exit(1)

        cells, voxel_size = load_flow_from_dsg(scene_graph_path, self.get_logger())
        if not cells:
            self.get_logger().warning(
                f"[flow_voxel_publisher] DSG at {scene_graph_path} has no flow.map metadata; "
                "either the run didn't enable the temporal module, or the artifact "
                "predates flow_map embedding. Idling."
            )
            return

        if voxel_size is None or voxel_size <= 0.0:
            voxel_size = float(self.declare_parameter("voxel_size", 0.1).value)
        if voxel_size <= 0.0:
            self.get_logger().error(
                f"[flow_voxel_publisher] non-positive voxel_size {voxel_size}; aborting"
            )
            sys.exit(1)

        frame_id = self.declare_parameter("frame_id", "map").value
        topic = self.declare_parameter("topic", "/hydra_dsg_visualizer/flow_voxels").value
        rate_hz = float(self.declare_parameter("rate_hz", 1.0).value)
        stride = int(self.declare_parameter("stride", 1).value)
        max_cells = int(self.declare_parameter("max_cells", 50000).value)

        self.get_logger().info(
            f"[flow_voxel_publisher] loaded {len(cells)} cells from DSG metadata "
            f"(voxel_size={voxel_size:.3f})"
        )

        # latched, as in ROS 1
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(MarkerArray, topic, qos)
        self._msg = build_markers(
            cells, voxel_size, frame_id, stride, max_cells, self.get_clock().now().to_msg()
        )
        self._pub.publish(self._msg)
        if rate_hz > 0.0:
            self._timer = self.create_timer(1.0 / rate_hz, self._republish)

    def _republish(self):
        # rebuild stamps so RViz doesn't drop them after a TF clock reset
        now = self.get_clock().now().to_msg()
        for marker in self._msg.markers:
            marker.header.stamp = now
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = FlowVoxelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
