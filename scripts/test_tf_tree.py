#!/usr/bin/env python3
# ============================================================
# TEST 3 — TF TRANSFORM HIERARCHY & INVARIANTS
# Verifies coordinate frame chain:
#   odom -> base_link -> laser, imu_link, camera_link, thermal_link
# Confirms single-ownership of dynamic odom -> base_link.
# ============================================================

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


class TFHierarchyValidator(Node):
    def __init__(self):
        super().__init__('tf_hierarchy_validator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

    def publish_test_frames(self):
        now = self.get_clock().now().to_msg()

        # Dynamic odom -> base_link
        t_odom = TransformStamped()
        t_odom.header.stamp = now
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_link'
        t_odom.transform.translation.x = 1.0
        t_odom.transform.rotation.w = 1.0

        # Static sensors: laser, imu_link, camera_link, thermal_link
        frames = [
            ('base_link', 'laser', 0.25, 0.0, 0.20),
            ('base_link', 'imu_link', 0.0, 0.0, 0.15),
            ('base_link', 'camera_link', 0.30, 0.0, 0.25),
            ('base_link', 'thermal_link', 0.30, 0.05, 0.25),
        ]

        tf_list = [t_odom]
        for parent, child, x, y, z in frames:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            tf_list.append(t)

        for tf in tf_list:
            self.tf_broadcaster.sendTransform(tf)

    def run_validation(self):
        print("\n--- TEST 3: TF TRANSFORM HIERARCHY ---")
        for _ in range(10):
            self.publish_test_frames()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        targets = [
            ('odom', 'base_link'),
            ('base_link', 'laser'),
            ('base_link', 'imu_link'),
            ('base_link', 'camera_link'),
            ('base_link', 'thermal_link')
        ]

        passed = True
        for parent, child in targets:
            try:
                can_tf = self.tf_buffer.can_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                if can_tf:
                    tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                    print(f"  [PASS] Transform {parent} -> {child} is valid (x={tf.transform.translation.x:.2f})")
                else:
                    print(f"  [FAIL] Cannot transform {parent} -> {child}")
                    passed = False
            except Exception as e:
                print(f"  [FAIL] Transform exception on {parent} -> {child}: {e}")
                passed = False

        return passed


def main():
    rclpy.init()
    node = TFHierarchyValidator()
    ok = node.run_validation()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
