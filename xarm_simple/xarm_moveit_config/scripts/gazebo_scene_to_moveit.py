#!/usr/bin/env python3

import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive
from tf_transformations import euler_matrix, euler_from_matrix, quaternion_from_matrix


@dataclass
class PoseRPY:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class IncludedModel:
    name: str
    uri: str
    pose: PoseRPY
    scale: Tuple[float, float, float]


def _parse_pose(text: str) -> PoseRPY:
    values = [float(v) for v in text.strip().split()]
    if len(values) == 6:
        return PoseRPY(*values)
    if len(values) == 3:
        x, y, z = values
        return PoseRPY(x, y, z, 0.0, 0.0, 0.0)
    raise ValueError(f"Unsupported pose format: '{text}'")


def _identity_pose() -> PoseRPY:
    return PoseRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _parse_scale(text: str) -> Tuple[float, float, float]:
    values = [float(v) for v in text.strip().split()]
    if len(values) == 1:
        s = values[0]
        return (s, s, s)
    if len(values) >= 3:
        return (values[0], values[1], values[2])
    raise ValueError(f"Unsupported scale format: '{text}'")


def _pose_to_matrix(pose: PoseRPY) -> np.ndarray:
    matrix = euler_matrix(pose.roll, pose.pitch, pose.yaw)
    matrix[0:3, 3] = [pose.x, pose.y, pose.z]
    return matrix


def _matrix_to_pose(matrix: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    qx, qy, qz, qw = quaternion_from_matrix(matrix)
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


def _matrix_to_poserp(matrix: np.ndarray) -> PoseRPY:
    roll, pitch, yaw = euler_from_matrix(matrix)
    return PoseRPY(
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        float(matrix[2, 3]),
        float(roll),
        float(pitch),
        float(yaw),
    )


def _scale_vector(vec: Sequence[float], scale: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (vec[0] * scale[0], vec[1] * scale[1], vec[2] * scale[2])


def _scale_radius(scale: Tuple[float, float, float]) -> float:
    return (scale[0] + scale[1]) * 0.5


def _compose_pose(parent: PoseRPY, child: PoseRPY) -> Pose:
    parent_matrix = _pose_to_matrix(parent)
    child_matrix = _pose_to_matrix(child)
    combined = np.matmul(parent_matrix, child_matrix)
    return _matrix_to_pose(combined)


def _table_primitives(include_pose: PoseRPY, scale: Tuple[float, float, float], frame_id: str, object_id: str) -> CollisionObject:
    collision_object = CollisionObject()
    collision_object.id = object_id
    collision_object.header.frame_id = frame_id

    top_dims = _scale_vector((1.5, 0.8, 0.03), scale)
    top_pose = _compose_pose(include_pose, PoseRPY(0.0, 0.0, 1.0 * scale[2], 0.0, 0.0, 0.0))
    top = SolidPrimitive()
    top.type = SolidPrimitive.BOX
    top.dimensions = list(top_dims)
    collision_object.primitives.append(top)
    collision_object.primitive_poses.append(top_pose)

    leg_positions = [
        (0.68, 0.38, 0.5),
        (0.68, -0.38, 0.5),
        (-0.68, -0.38, 0.5),
        (-0.68, 0.38, 0.5),
    ]

    radius_scale = max(_scale_radius(scale), sys.float_info.epsilon)
    length_scale = max(scale[2], sys.float_info.epsilon)

    for x, y, z in leg_positions:
        leg_pose = _compose_pose(include_pose, PoseRPY(x * scale[0], y * scale[1], z * scale[2], 0.0, 0.0, 0.0))
        leg = SolidPrimitive()
        leg.type = SolidPrimitive.CYLINDER
        leg.dimensions = [1.0 * length_scale, 0.02 * radius_scale]
        collision_object.primitives.append(leg)
        collision_object.primitive_poses.append(leg_pose)

    collision_object.operation = CollisionObject.ADD
    return collision_object


def _load_world_models(world_path: str) -> List[IncludedModel]:
    tree = ET.parse(world_path)
    root = tree.getroot()
    includes: List[IncludedModel] = []
    for include in root.findall(".//include"):
        uri_elem = include.find("uri")
        if uri_elem is None or not uri_elem.text:
            continue
        uri = uri_elem.text.strip()
        pose_elem = include.find("pose")
        pose = _parse_pose(pose_elem.text) if pose_elem is not None and pose_elem.text else _identity_pose()
        scale_elem = include.find("scale")
        scale = _parse_scale(scale_elem.text) if scale_elem is not None and scale_elem.text else (1.0, 1.0, 1.0)
        name_elem = include.find("name")
        name = name_elem.text.strip() if name_elem is not None and name_elem.text else os.path.basename(uri)
        includes.append(IncludedModel(name=name, uri=uri, pose=pose, scale=scale))
    return includes


class GazeboSceneToMoveIt(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_scene_to_moveit")

        self.declare_parameter("world_path", Parameter.Type.STRING)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("collision_object_prefix", "")
        self.declare_parameter("ignored_models", [])
        self.declare_parameter("synchronize_table", True)
        self.declare_parameter("gazebo_reference_pose", "")

        self._world_path = self.get_parameter("world_path").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value or "world"
        self._prefix = self.get_parameter("collision_object_prefix").get_parameter_value().string_value

        ignored = self.get_parameter("ignored_models").get_parameter_value()
        self._ignored_models = set(ignored.string_array_value if ignored.type == Parameter.Type.STRING_ARRAY else [])
        self._sync_table = self.get_parameter("synchronize_table").get_parameter_value().bool_value
        reference_pose_raw = self.get_parameter("gazebo_reference_pose").get_parameter_value().string_value
        self._reference_inverse = None
        if reference_pose_raw:
            try:
                ref_values = [float(v) for v in reference_pose_raw.split()]
                if len(ref_values) != 6:
                    raise ValueError(f"Expected 6 values, got {len(ref_values)}")
                ref_pose = PoseRPY(*ref_values)
                ref_matrix = _pose_to_matrix(ref_pose)
                self._reference_inverse = np.linalg.inv(ref_matrix)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"Failed to parse gazebo reference pose '{reference_pose_raw}': {exc}. "
                    "Objects will be aligned directly with Gazebo world."
                )

        if not self._world_path:
            self.get_logger().warn("Parameter 'world_path' is empty; no scene objects will be added.")
            self._collision_objects: List[CollisionObject] = []
        elif not os.path.exists(self._world_path):
            self.get_logger().warn(f"World file '{self._world_path}' not found; skipping scene synchronization.")
            self._collision_objects = []
        else:
            try:
                includes = _load_world_models(self._world_path)
                self._collision_objects = self._build_collision_objects(includes)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to parse world '{self._world_path}': {exc}")
                self._collision_objects = []

        self._apply_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self._request_future = None
        self._timer = self.create_timer(1.0, self._try_apply_scene)

        if self._collision_objects:
            for obj in self._collision_objects:
                self.get_logger().info(f"Queued collision object '{obj.id}' from Gazebo scene.")
        else:
            self.get_logger().info("No Gazebo scene collision objects queued for MoveIt.")

    def _build_collision_objects(self, includes: Iterable[IncludedModel]) -> List[CollisionObject]:
        objects: List[CollisionObject] = []
        for model in includes:
            if model.name in self._ignored_models:
                self.get_logger().debug(f"Skipping ignored model '{model.name}'.")
                continue

            uri_lower = model.uri.lower()
            object_id = f"{self._prefix}{model.name}" if self._prefix else model.name
            pose = self._transform_pose_to_moveit(model.pose)

            if self._sync_table and ("table" in uri_lower or model.name.lower() == "table"):
                collision_object = _table_primitives(pose, model.scale, self._frame_id, object_id)
                objects.append(collision_object)
                continue

        return objects

    def _transform_pose_to_moveit(self, pose: PoseRPY) -> PoseRPY:
        if self._reference_inverse is None:
            return pose
        pose_matrix = _pose_to_matrix(pose)
        transformed = np.matmul(self._reference_inverse, pose_matrix)
        return _matrix_to_poserp(transformed)

    def _try_apply_scene(self) -> None:
        if not self._collision_objects:
            self._timer.cancel()
            return

        if self._request_future is None:
            if not self._apply_client.wait_for_service(timeout_sec=0.0):
                self.get_logger().debug("Waiting for 'apply_planning_scene' service...")
                return
            request = ApplyPlanningScene.Request()
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects = self._collision_objects
            request.scene = scene
            self._request_future = self._apply_client.call_async(request)
            return

        if self._request_future.done():
            try:
                response = self._request_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to apply planning scene: {exc}")
            else:
                if response and response.success:
                    self.get_logger().info("Gazebo scene collision objects applied to MoveIt planning scene.")
                else:
                    self.get_logger().error("ApplyPlanningScene service returned failure.")
            finally:
                self._timer.cancel()


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboSceneToMoveIt()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
