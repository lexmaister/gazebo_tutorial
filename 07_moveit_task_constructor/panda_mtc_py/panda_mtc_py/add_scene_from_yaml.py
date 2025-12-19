#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


NODE = "add_scene_from_yaml"
logger = get_logger(NODE)


def decode_primitive_type(value) -> int:
    """Return a SolidPrimitive enum from int or string; fallback to BOX."""
    if isinstance(value, int):
        return value

    if not isinstance(value, str):
        return SolidPrimitive.BOX

    match value.upper():
        case "BOX":
            return SolidPrimitive.BOX
        case "SPHERE":
            return SolidPrimitive.SPHERE
        case "CYLINDER":
            return SolidPrimitive.CYLINDER
        case "CONE":
            return SolidPrimitive.CONE
        case _:
            return SolidPrimitive.BOX


def dict_to_collision_object(data: dict) -> CollisionObject:
    """Convert a dict parsed from YAML into a CollisionObject message."""
    msg = CollisionObject()

    header_dict = data.get("header", {})
    msg.header = Header()
    msg.header.frame_id = header_dict.get("frame_id", "")

    msg.id = data.get("id", "")

    msg.primitives = []
    for prim_dict in data.get("primitives", []):
        prim = SolidPrimitive()
        prim.type = decode_primitive_type(prim_dict.get("type", "BOX"))
        prim.dimensions = prim_dict.get("dimensions", [])
        msg.primitives.append(prim)

    msg.primitive_poses = []
    for pose_dict in data.get("primitive_poses", []):
        pose = Pose()
        pos = pose_dict.get("position", {})
        ori = pose_dict.get("orientation", {})
        pose.position.x = pos.get("x", 0.0)
        pose.position.y = pos.get("y", 0.0)
        pose.position.z = pos.get("z", 0.0)
        pose.orientation.x = ori.get("x", 0.0)
        pose.orientation.y = ori.get("y", 0.0)
        pose.orientation.z = ori.get("z", 0.0)
        pose.orientation.w = ori.get("w", 1.0)
        msg.primitive_poses.append(pose)

    msg.operation = CollisionObject.ADD

    return msg


class AddSceneFromYaml(Node):
    """Node that publishes a collision object described in a YAML file."""

    def __init__(self) -> None:
        super().__init__(NODE)

        self.declare_parameter("scene_path", "")
        self.scene_path = (
            self.get_parameter("scene_path").get_parameter_value().string_value
        )

        if not self.scene_path:
            raise RuntimeError('Parameter "scene_path" is empty')
        if not os.path.exists(self.scene_path):
            raise RuntimeError(f"YAML file not found: {self.scene_path}")

        self.subscriber = self.create_subscription(
            msg_type=PlanningScene,
            topic="/monitored_planning_scene",
            callback=self._on_planning_scene,
            qos_profile=10,
        )

        self.publisher = self.create_publisher(
            msg_type=CollisionObject,
            topic="/collision_object",
            qos_profile=10,
        )

        self.timer = self.create_timer(2.0, self._on_timer, autostart=False)

        # publish once
        self._published = False
        # to check if object was added to scene
        self.object_id = ""

        logger.info(f"Will load scene from: {self.scene_path}")
        # to init subscriber
        self.timer.reset()

    def _on_timer(self) -> None:
        """Load YAML and publish a CollisionObject once."""
        if self._published:
            return

        try:
            with open(self.scene_path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as exc:
            raise RuntimeError(f"Failed to read YAML file: {exc}")

        try:
            msg = dict_to_collision_object(data)
            self.object_id = msg.id
            msg.header.stamp = self.get_clock().now().to_msg()
        except Exception as exc:
            raise RuntimeError(f"Failed to convert YAML to CollisionObject: {exc}")

        logger.info(
            f'Publishing collision object: id="{msg.id}", '
            f'frame="{msg.header.frame_id}" on /collision_object'
        )
        self.publisher.publish(msg)
        self._published = True

    def _on_planning_scene(self, msg: PlanningScene):
        """Callback for handling messages in /monitored_planning_scene topic"""
        logger.info(f"Got msg in 'monitored_planning_scene' topic: {msg.name}")
        c_objects: list[CollisionObject] = list(msg.world.collision_objects)
        for co in c_objects:
            if co.id == self.object_id:
                logger.info(f"Collision object {co.id!r} is added")
                rclpy.shutdown()


def main(args=None) -> None:
    """Initialize and spin the AddSceneFromYaml node."""
    rclpy.init(args=args)
    node = None
    try:
        node = AddSceneFromYaml()
        rclpy.spin(node)
    except Exception as e:
        logger.error(str(e))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
