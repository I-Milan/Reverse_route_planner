#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
from transforms3d.euler import euler2quat


class TwistToTFNode(Node):
    def __init__(self):
        super().__init__('twist_to_tf_node')

        # Twist üzenetek fogadása
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Frame nevek
        self.parent_frame = 'map'
        self.child_frame = 'base_link'

        # Állapotváltozók
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_update_time = self.get_clock().now()
        self.last_twist_time = self.get_clock().now()

        # Timer, ami folyamatosan frissíti a pozíciót
        self.timer = self.create_timer(0.05, self.update_tf)  # 50 ms

        self.get_logger().info(" TwistToTF node elindult — figyeli a /cmd_vel üzeneteket.")

    def twist_callback(self, msg: Twist):
        """Mentjük az utolsó ismert sebességeket."""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        self.last_twist_time = self.get_clock().now()

    def update_tf(self):
        """Folyamatosan frissíti a TF-et az utolsó ismert sebességek alapján."""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        # Ha 1 másodpercnél régebbi a Twist, nullázzuk a sebességet
        time_since_twist = (now - self.last_twist_time).nanoseconds / 1e9
        if time_since_twist > 1.0:
            v = 0.0
            w = 0.0
        else:
            v = self.linear_vel
            w = self.angular_vel

        # Dead reckoning integráció
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Euler → Quaternion
        q = euler2quat(0, 0, self.theta)

        # TF létrehozása
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        # TF publikálása
        self.tf_broadcaster.sendTransform(t)

        # Opcionális logolás ritkábban
        if int(now.nanoseconds * 1e-9) % 2 == 0:  # kb. 2 mp-enként
            self.get_logger().info(
                f"x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.theta):.1f}° (v={v:.2f}, w={w:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
