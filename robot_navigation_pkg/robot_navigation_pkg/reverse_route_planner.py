#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from tf2_ros import Buffer, TransformListener
import tf_transformations


# ============================================================
# Segédfüggvények
# ============================================================

def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def hypot(dx, dy):
    return math.hypot(dx, dy)


# ============================================================
# Fő Node
# ============================================================

class FeasiblePlannerFSM(Node):

    def __init__(self):
        super().__init__("feasible_planner_fsm")

        # ===== Robot paraméterek =====
        self.track_width = 0.63
        self.v_forward = 0.5
        self.v_reverse = -0.5

        # ===== Paraméterek =====
        self.reach_tol = 0.10
        self.min_reverse_dist = 0.20
        self.heading_kp = 2.0
        self.switch_delay = 0.5

        # ===== Forward lassítás =====
        self.angle_threshold = math.radians(15.0)
        self.slow_radius_outer = 0.5
        self.slow_radius_inner = 0.25
        self.slow_factor_outer = 0.65
        self.slow_factor_inner = 0.35

        # ===== ROS =====
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(PoseArray, "pursuitgoal", self.wp_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.step)

        # ===== Waypoint állapot =====
        self.wps = None
        self.reached = []
        self.active_target_idx = None

        # ===== FSM állapot =====
        self.state = "FORWARD"
        self.state_start_time = 0.0

        # ===== Reverse állapot =====
        self.reverse_speed = -0.25
        self.reverse_travel = 0.0
        self.last_time = None

    # ============================================================

    def wp_cb(self, msg):
        if not msg.poses:
            return
        self.wps = msg
        if not self.reached:
            self.reached = [False] * len(msg.poses)

    # ============================================================

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.now()
        self.last_time = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================

    def step(self):

        if self.wps is None:
            self.publish(0.0, 0.0)
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.wps.header.frame_id,
                "base_link",
                rclpy.time.Time()
            )
        except Exception:
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y

        q = tf.transform.rotation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        if self.active_target_idx is None:
            self.active_target_idx = self.choose_target(rx, ry)
            if self.active_target_idx is None:
                self.publish(0.0, 0.0)
                return

        idx = self.active_target_idx
        tx = self.wps.poses[idx].position.x
        ty = self.wps.poses[idx].position.y

        dx, dy = tx - rx, ty - ry
        d = hypot(dx, dy)
        alpha = wrap(math.atan2(dy, dx) - yaw)

        if d <= self.reach_tol:
            self.reached[idx] = True
            self.active_target_idx = None
            self.publish(0.0, 0.0)
            return

        theta_max = max((2.0 * d / self.track_width),
                        math.radians(5.0))
        feasible = abs(alpha) <= theta_max

        now = self.now()

        # ============================================================
        # FSM
        # ============================================================

        # ---------------- FORWARD ----------------
        if self.state == "FORWARD":

            if not feasible:
                self.change_state("FORWARD_TO_REVERSE_DELAY")
                self.publish(0.0, 0.0)
                return

            v = self.compute_forward_speed(d, alpha)
            w = self.compute_angular(v, alpha)
            self.publish(v, w)
            return

        # ---------------- DELAY F->R ----------------
        if self.state == "FORWARD_TO_REVERSE_DELAY":

            if now - self.state_start_time >= self.switch_delay:
                self.reverse_speed = -0.25
                self.reverse_travel = 0.0
                self.change_state("REVERSE_ACCEL")

            self.publish(0.0, 0.0)
            return

        # ---------------- REVERSE ACCEL ----------------
        if self.state == "REVERSE_ACCEL":

            if now - self.state_start_time >= 0.2:
                self.reverse_speed = self.v_reverse
                self.change_state("REVERSE_CRUISE")

            self.integrate_reverse(now)
            w = self.compute_angular(self.reverse_speed, alpha)
            self.publish(self.reverse_speed, w)
            return

        # ---------------- REVERSE CRUISE ----------------
        if self.state == "REVERSE_CRUISE":

            self.integrate_reverse(now)

            if feasible and self.reverse_travel >= self.min_reverse_dist:
                self.change_state("REVERSE_SLOW_PHASE")

            w = self.compute_angular(self.reverse_speed, alpha)
            self.publish(self.reverse_speed, w)
            return

        # ---------------- REVERSE SLOW ----------------
        if self.state == "REVERSE_SLOW_PHASE":

            T = 0.5
            elapsed = now - self.state_start_time

            if elapsed >= T:
                self.change_state("REVERSE_TO_FORWARD_DELAY")
                self.publish(0.0, 0.0)
                return

            ratio = 1.0 - (elapsed / T)
            v = self.v_reverse * 0.5 * ratio
            w = self.compute_angular(v, alpha)
            self.publish(v, w)
            return

        # ---------------- DELAY R->F ----------------
        if self.state == "REVERSE_TO_FORWARD_DELAY":

            if now - self.state_start_time >= self.switch_delay:
                self.change_state("FORWARD")

            self.publish(0.0, 0.0)
            return

    # ============================================================

    def compute_forward_speed(self, d, alpha):

        v = self.v_forward

        if abs(alpha) > self.angle_threshold:

            if d <= self.slow_radius_inner:
                scale = self.slow_factor_inner

            elif d <= self.slow_radius_outer:
                ratio = ((d - self.slow_radius_inner) /
                         (self.slow_radius_outer -
                          self.slow_radius_inner))
                scale = (self.slow_factor_inner +
                         ratio *
                         (self.slow_factor_outer -
                          self.slow_factor_inner))
            else:
                scale = 1.0

            v *= scale

        return v

    def compute_angular(self, v, alpha):

        w_max = (2.0 * abs(v)) / self.track_width
        return w_max * math.tanh(self.heading_kp * alpha)

    def integrate_reverse(self, now):

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        self.reverse_travel += abs(self.reverse_speed) * dt

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def choose_target(self, rx, ry):
        best, best_d = None, float("inf")
        for i, p in enumerate(self.wps.poses):
            if self.reached[i]:
                continue
            d = hypot(p.position.x - rx, p.position.y - ry)
            if d < best_d:
                best_d, best = d, i
        return best


# ============================================================

def main():
    rclpy.init()
    node = FeasiblePlannerFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()