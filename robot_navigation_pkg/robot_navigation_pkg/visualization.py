#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import math

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer_pursuitgoal')

        # --- Paraméterek ---
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('waypoint_topic', 'pursuitgoal')
        self.declare_parameter('marker_topic', 'visualization_marker_array')

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.waypoint_topic = self.get_parameter('waypoint_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value

        # --- Waypoint lista ---
        
        self.waypoints = [
            (0.00, 1.25), (0.00, -1.25),
            (0.51, 1.31), (0.51, -1.19),
            (1.01, 1.36), (1.01, -1.14),
            (1.52, 1.40), (1.52, -1.10),
            (2.02, 1.45), (2.02, -1.05),
            (2.53, 1.49), (2.53, -1.01),
            (3.03, 1.53), (3.03, -0.97),
            (3.54, 1.57), (3.54, -0.93),
            (4.04, 1.61), (4.04, -0.89),
            (4.55, 1.65), (4.55, -0.85),
            (5.05, 1.69), (5.05, -0.81),
            (5.56, 1.73), (5.56, -0.77),
            (6.06, 1.77), (6.06, -0.73),
            (6.57, 1.80), (6.57, -0.70),
            (7.07, 1.84), (7.07, -0.66),
            (7.58, 1.88), (7.58, -0.62),
            (8.08, 1.92), (8.08, -0.58),
            (8.59, 1.95), (8.59, -0.55),
            # (9.09, 1.99), (9.09, -0.51),
            # (9.60, 2.02), (9.60, -0.48),
            # (10.10, 2.06), (10.10, -0.44),
            # (10.61, 2.09), (10.61, -0.41),
            # (11.11, 2.13), (11.11, -0.37),
            # (11.62, 2.16), (11.62, -0.34),
            # (12.12, 2.19), (12.12, -0.31),
            # (12.63, 2.23), (12.63, -0.27),
            # (13.13, 2.26), (13.13, -0.24),
            # (13.64, 2.29), (13.64, -0.21),
            # (14.14, 2.32), (14.14, -0.18),
            # (14.65, 2.35), (14.65, -0.15),
            # (15.15, 2.38), (15.15, -0.12),
            # (15.66, 2.41), (15.66, -0.09),
            # (16.16, 2.43), (16.16, -0.07),
            # (16.67, 2.46), (16.67, -0.04),
            # (17.17, 2.48), (17.17, -0.02),
            # (17.68, 2.50), (17.68, 0.00),
            # (18.18, 2.52), (18.18, 0.02),
            # (18.69, 2.54), (18.69, 0.04),
        ]

        # --- Eredeti, kikommentelt waypoint-listák megőrzése ---
        '''
        
        self.waypoints = [
            (0.0, 0.0),
            (5.0, 0.0),
            (5.39, 2.25),
            (3.66, 4.15),
            (0.0, 4.33),
            (-2.48, 2.07),
            (-3.0, -1.5),
            (-1.12, -4.42),
            (2.39, -5.18),
            (5.53, -3.35),
            (6.6, 0.02),
            (4.93, 3.06),
            (1.52, 4.33),
            (-1.55, 3.14),
            (-3.0, 0.0),
            (-2.25, -2.92),
            (0.06, -4.24),
            (2.58, -3.64),
            (3.96, -1.34),
            (3.5, 1.0),
            (1.58, 2.0),
            (-0.42, 1.08)
        ]
        
        
        self.waypoints = [
            (0.0, 0.0),
            (1.0, 0.5),
            (2.0, 1.0),
            (3.0, 1.5),
            (4.0, 2.0),
            (5.0, 2.2),
            (6.0, 2.4),
            (7.0, 2.5),
            (8.0, 2.6),
            (9.0, 2.5),
            (10.0, 2.4),
            (11.0, 2.0),
            (12.0, 1.5),
            (13.0, 1.0),
            (14.0, 0.5),
            (15.0, 0.0)
        ]
        '''

        # --- TF listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publisher-ek ---
        self.pose_pub = self.create_publisher(PoseArray, self.waypoint_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        # --- Belső lista a megtett úthoz ---
        self.path_points = []

        # --- Timerek ---
        self.create_timer(1.0, self.publish_waypoints)   # Waypoints 1 Hz
        self.create_timer(0.1, self.publish_markers)     # Marker 10 Hz

        # --- Távolság és szög kiszámítás ---
        self.compute_turn_angles_and_distances()
        self.get_logger().info("✅ Path Visualizer (pursuitgoal + markers) started")

    def compute_turn_angles_and_distances(self):
        self.get_logger().info("📏 Távolságok és 📐 Szögek a pontok között:")
        for i in range(1, len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i - 1]
            x2, y2 = self.waypoints[i]
            x3, y3 = self.waypoints[i + 1]

            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            angle1 = math.atan2(y2 - y1, x2 - x1)
            angle2 = math.atan2(y3 - y2, x3 - x2)
            delta_angle = (angle2 - angle1 + math.pi) % (2 * math.pi) - math.pi
            delta_deg = math.degrees(delta_angle)

            self.get_logger().info(
                f"  {i}. szakasz: távolság = {distance:.2f} m, fordulás = {delta_deg:.2f}°"
            )

    def publish_waypoints(self):
        msg = PoseArray()
        msg.header.frame_id = self.map_frame
        for x, y in self.waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            msg.poses.append(pose)
        self.pose_pub.publish(msg)

    def publish_markers(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            self.path_points.append((x, y))
        except Exception:
            pass

        t_now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = self.map_frame
        waypoint_marker.header.stamp = t_now
        waypoint_marker.ns = "waypoints"
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.15
        waypoint_marker.scale.y = 0.15
        waypoint_marker.scale.z = 0.15
        waypoint_marker.color.r = 1.0
        waypoint_marker.color.g = 0.6
        waypoint_marker.color.b = 0.0
        waypoint_marker.color.a = 1.0
        for (wx, wy) in self.waypoints:
            pt = Point()
            pt.x = wx
            pt.y = wy
            waypoint_marker.points.append(pt)
        marker_array.markers.append(waypoint_marker)

        path_marker = Marker()
        path_marker.header.frame_id = self.map_frame
        path_marker.header.stamp = t_now
        path_marker.ns = "path_line"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        for (px, py) in self.path_points:
            pt = Point()
            pt.x = px
            pt.y = py
            path_marker.points.append(pt)
        marker_array.markers.append(path_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


