#!/usr/bin/env python3
"""
Stream the right VR controller's motion to a Cartesian impedance controller
(tiago_pro_custom_controllers/CartesianImpedanceController).

The arm follows the controller's *change* in pose, starting from the end effector's current
pose (read from TF), so the position of the VR tracking origin does not matter. Commands go to
/<controller>/desired_pose (geometry_msgs/Pose in the controller's root_link frame).

Safety:
  * Starts disabled: nothing is published until the Unity "toggle tracking" button is pressed
    (service toggle_tracking). The "stop robot" button (service stop_robot) disables it again;
    the controller then holds the last pose.
  * Tracking dropouts (zero or stale controller pose) freeze the target and re-latch on return.
  * The target is clamped to a box / rotation cone around the pose at enable and slew-rate limited.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener, TransformException

from custom_teleop_messages.msg import XRMod
from custom_teleop_messages.srv import BoolTrigger


def to_np(pose):
    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    rot = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w])
    return pos, rot


class VRCartesianTeleop(Node):

    def __init__(self):
        super().__init__('vr_cartesian_teleop')

        self.declare_parameter('side', 'right')
        self.declare_parameter('root_frame', 'torso_lift_link')
        self.declare_parameter('ee_frame', 'arm_right_7_link')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('vr_yaw', 0.0)               # rotation of the VR forward axis vs robot x [rad]
        self.declare_parameter('position_scale', 1.0)       # robot motion per unit controller motion
        self.declare_parameter('max_translation', 0.25)     # per axis from the pose at enable [m]
        self.declare_parameter('max_rotation', 1.0)         # from the pose at enable [rad]
        self.declare_parameter('max_linear_speed', 0.3)     # [m/s]
        self.declare_parameter('max_angular_speed', 1.0)    # [rad/s]
        self.declare_parameter('tracking_timeout', 0.2)     # [s] older controller data counts as lost
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('output_topic', '')          # default: the controller's desired_pose
        self.declare_parameter('translation_only', False)   # keep the orientation, follow only the position
        self.declare_parameter('debug_viz', True)           # publish RViz debug topics under ~/debug
        self.declare_parameter('trail_length', 300)         # points kept per trail

        side = self.get_parameter('side').value
        self.root_frame = self.get_parameter('root_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        rate = self.get_parameter('rate').value
        self.position_scale = self.get_parameter('position_scale').value
        self.max_translation = self.get_parameter('max_translation').value
        self.max_rotation = self.get_parameter('max_rotation').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.tracking_timeout = self.get_parameter('tracking_timeout').value
        self.translation_only = self.get_parameter('translation_only').value
        self.align = Rotation.from_euler('z', self.get_parameter('vr_yaw').value)
        self.side = side

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        output_topic = self.get_parameter('output_topic').value or \
            f'/cartesian_impedance_controller_{side}/desired_pose'
        self.pose_pub = self.create_publisher(Pose, output_topic, 10)
        self.debug_viz = self.get_parameter('debug_viz').value
        self.trail_length = self.get_parameter('trail_length').value
        self.target_pub = self.create_publisher(PoseStamped, '/vr_cartesian/debug/target', 10)
        self.command_pub = self.create_publisher(PoseStamped, '/vr_cartesian/debug/command', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/vr_cartesian/debug/trails', 10)
        self.trails = {'target': [], 'command': [], 'actual': []}
        self.last_viz = None
        self.xr_sub = self.create_subscription(XRMod, '/xr_input', self.xr_callback, 1)
        self.create_service(BoolTrigger, 'toggle_tracking', self.toggle_callback)
        self.create_service(BoolTrigger, 'stop_robot', self.stop_callback)

        self.enabled = False
        self.vr = None              # (position, rotation, receive time)
        self.ref = None             # latch: controller start pose and target start pose
        self.origin = None          # pose at enable, centre of the clamp
        self.cmd = None             # slew-limited command
        self.last_tick = None

        self.timer = self.create_timer(1.0 / rate, self.tick)
        if self.get_parameter('start_enabled').value:
            self.enable()
        self.get_logger().info(
            f'VR Cartesian teleop for the {side} arm ready ({"translation only" if self.translation_only else "position and orientation"}); '
            'disabled until toggle_tracking is called.')

    # --- inputs ---------------------------------------------------------------------------------
    def xr_callback(self, msg):
        pose = msg.right_controller_pose if self.side == 'right' else msg.left_controller_pose
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        position = [pose.position.x, pose.position.y, pose.position.z]
        # untracked controllers report the origin and an all-zero quaternion
        if sum(abs(v) for v in position) < 0.001 or np.linalg.norm(quat) < 1e-6:
            self.vr = None
            return
        pos, rot = to_np(pose)
        self.vr = (pos, rot, self.get_clock().now())

    def toggle_callback(self, request, response):
        if self.enabled:
            self.disable('toggle_tracking')
        else:
            self.enable()
        response.state = self.enabled
        return response

    def stop_callback(self, request, response):
        self.disable('stop_robot')
        response.state = self.enabled
        return response

    # --- state ------------------------------------------------------------------------------------
    def enable(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.root_frame, self.ee_frame, Time())
        except TransformException as ex:
            self.get_logger().error(f'Cannot enable: no transform {self.root_frame} -> {self.ee_frame}: {ex}')
            return
        t, q = tf.transform.translation, tf.transform.rotation
        pos = np.array([t.x, t.y, t.z])
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        self.origin = (pos, rot)
        self.cmd = (pos.copy(), rot)
        self.ref = None
        self.last_tick = None
        self.enabled = True
        for trail in self.trails.values():
            trail.clear()
        self.get_logger().info(f'Enabled. Start pose {pos.round(3).tolist()} in {self.root_frame}.')

    def disable(self, reason):
        if self.enabled:
            self.get_logger().info(f'Disabled ({reason}); the controller holds the last commanded pose.')
        self.enabled = False
        self.ref = None

    # --- control loop ---------------------------------------------------------------------------
    def tick(self):
        if not self.enabled:
            return
        now = self.get_clock().now()
        dt = 0.0 if self.last_tick is None else (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now

        tracked = self.vr is not None and \
            (now - self.vr[2]).nanoseconds * 1e-9 < self.tracking_timeout
        target = None
        if not tracked:
            self.ref = None  # freeze the command; re-latch when tracking returns
        else:
            vr_pos, vr_rot, _ = self.vr
            if self.ref is None:
                self.ref = (vr_pos.copy(), vr_rot, self.cmd[0].copy(), self.cmd[1])
            vr_pos0, vr_rot0, base_pos, base_rot = self.ref

            desired_pos = base_pos + self.align.apply(vr_pos - vr_pos0) * self.position_scale
            delta_rot = self.align * (vr_rot * vr_rot0.inv()) * self.align.inv()
            desired_rot = base_rot if self.translation_only else delta_rot * base_rot

            origin_pos, origin_rot = self.origin
            desired_pos = np.clip(desired_pos, origin_pos - self.max_translation,
                                  origin_pos + self.max_translation)
            away = (desired_rot * origin_rot.inv()).as_rotvec()
            angle = np.linalg.norm(away)
            if angle > self.max_rotation:
                desired_rot = Rotation.from_rotvec(away * self.max_rotation / angle) * origin_rot

            target = (desired_pos, desired_rot)
            self.cmd = self.slew(self.cmd, target, dt)

        self.publish(self.cmd)
        if self.debug_viz:
            self.visualize(now, target)

    def visualize(self, now, desired):
        if self.last_viz is not None and (now - self.last_viz).nanoseconds < 33e6:
            return
        self.last_viz = now
        stamp = now.to_msg()

        def stamped(pos, rot):
            msg = PoseStamped()
            msg.header.stamp, msg.header.frame_id = stamp, self.root_frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos.tolist()
            q = rot.as_quat().tolist()
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
            return msg

        # target: where the VR controller asks the end effector to be (before clamp and slew)
        if desired is not None:
            self.target_pub.publish(stamped(*desired))
            self.add_trail('target', desired[0])
        self.command_pub.publish(stamped(*self.cmd))
        self.add_trail('command', self.cmd[0])
        try:
            t = self.tf_buffer.lookup_transform(self.root_frame, self.ee_frame, Time()).transform.translation
            self.add_trail('actual', np.array([t.x, t.y, t.z]))
        except TransformException:
            pass

        colors = {'target': (1.0, 0.2, 0.2), 'command': (0.2, 1.0, 0.2), 'actual': (0.3, 0.5, 1.0)}
        markers = MarkerArray()
        for i, (name, points) in enumerate(self.trails.items()):
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp, self.root_frame
            m.ns, m.id, m.type, m.action = name, i, Marker.LINE_STRIP, Marker.ADD
            m.scale.x = 0.006
            m.color.r, m.color.g, m.color.b = colors[name]
            m.color.a = 1.0
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
            markers.markers.append(m)
        # box = clamp limits around the pose at enable
        box = Marker()
        box.header.stamp, box.header.frame_id = stamp, self.root_frame
        box.ns, box.id, box.type, box.action = 'limits', 10, Marker.CUBE, Marker.ADD
        o = self.origin[0]
        box.pose.position.x, box.pose.position.y, box.pose.position.z = o.tolist()
        box.pose.orientation.w = 1.0
        box.scale.x = box.scale.y = box.scale.z = 2.0 * self.max_translation
        box.color.r = box.color.g = box.color.b = 1.0
        box.color.a = 0.08
        markers.markers.append(box)
        self.marker_pub.publish(markers)

    def add_trail(self, name, pos):
        trail = self.trails[name]
        trail.append(pos.copy())
        del trail[:-self.trail_length]

    def slew(self, cmd, desired, dt):
        pos, rot = cmd
        step = desired[0] - pos
        max_step = self.max_linear_speed * dt
        dist = np.linalg.norm(step)
        if dist > max_step:
            step *= max_step / dist if dist > 0 else 0.0
        rot_step = (desired[1] * rot.inv()).as_rotvec()
        max_rot = self.max_angular_speed * dt
        angle = np.linalg.norm(rot_step)
        if angle > max_rot:
            rot_step *= max_rot / angle
        return pos + step, Rotation.from_rotvec(rot_step) * rot

    def publish(self, cmd):
        pos, rot = cmd
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = pos.tolist()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = rot.as_quat().tolist()
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = VRCartesianTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
