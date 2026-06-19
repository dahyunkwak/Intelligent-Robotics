#!/usr/bin/env python3
"""
ground_truth_pose_publisher.py

Gazebo /model_states 에서 turtlebot3_burger의 실제 위치를 읽어
Nav2가 사용하는 /amcl_pose 와 /tf (odom→base_footprint) 로 퍼블리시한다.

AMCL 없이 ground-truth 위치로 Nav2를 구동하기 위한 노드.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
)
from tf2_ros import TransformBroadcaster


class GroundTruthPosePublisher(Node):
    def __init__(self):
        super().__init__(
            'gt_pose_publisher',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )

        # ── 퍼블리셔 ──────────────────────────────────────
        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 구독 ──────────────────────────────────────────
        self.create_subscription(
            ModelStates, '/model_states', self._cb, 10
        )

        self.get_logger().info('✅ GroundTruthPosePublisher 시작')

    def _cb(self, msg: ModelStates):
        # turtlebot3_burger 인덱스 찾기
        try:
            idx = msg.name.index('turtlebot3_burger')
        except ValueError:
            return

        pose = msg.pose[idx]
        now  = self.get_clock().now().to_msg()

        # ── /amcl_pose 퍼블리시 ───────────────────────────
        amcl = PoseWithCovarianceStamped()
        amcl.header.stamp    = now
        amcl.header.frame_id = 'map'
        amcl.pose.pose       = pose
        # covariance를 0에 가깝게 → AMCL이 위치 확신
        amcl.pose.covariance[0]  = 0.001
        amcl.pose.covariance[7]  = 0.001
        amcl.pose.covariance[35] = 0.001
        self.amcl_pub.publish(amcl)

        # ── map → odom TF 퍼블리시 ────────────────────────
        # ground-truth 사용 시 map=odom 으로 처리 (offset 없음)
        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp            = now
        tf_map_odom.header.frame_id         = 'map'
        tf_map_odom.child_frame_id          = 'odom'
        tf_map_odom.transform.rotation.w    = 1.0  # 동일 프레임
        self.tf_broadcaster.sendTransform(tf_map_odom)


def main():
    rclpy.init()
    node = GroundTruthPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
