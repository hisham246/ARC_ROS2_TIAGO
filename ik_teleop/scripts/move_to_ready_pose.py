#!/usr/bin/env python3
"""
Slowly move an arm to a "ready" joint configuration from wherever it currently is, using the
standard joint trajectory controller, then (optionally) hand the arm over to the Cartesian
impedance controller.

Sequence:
  1. Read the current joint positions and print the planned move (current, target, duration).
  2. Ask for confirmation (skip with --yes).
  3. If the Cartesian impedance controller is active, switch back to the joint trajectory controller.
  4. Send ONE trajectory point to the arm's FollowJointTrajectory action. The duration is set from the
     largest joint change and --max-joint-speed, so the move is slow. Ctrl-C cancels the goal.
  5. Switch to the Cartesian impedance controller again (unless --no-switch).

SAFETY: the motion is a straight line in joint space. It is NOT collision checked and it is not
guaranteed to avoid the torso, the other arm or anything in the workspace. Keep the area clear, keep
the e-stop in reach, and start with a low --max-joint-speed on the real robot.

The default target is the nullspace posture of cartesian_impedance_controller_configuration_right.yaml
(tiago_pro_custom_controllers), so the impedance controller does not pull the arm away from it.

Usage:
    ros2 run ik_teleop move_to_ready_pose.py                       # sim or robot, asks first
    ros2 run ik_teleop move_to_ready_pose.py --max-joint-speed 0.15 --ros-args -p use_sim_time:=true
"""

import argparse
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# nullspace_pose of cartesian_impedance_controller_configuration_right.yaml
DEFAULT_READY_RIGHT = [-1.863, -1.1827, 0.48339, -1.5989, 0.2603, -1.0713, 1.16829]
# a peak/average velocity ratio of about 2 is assumed for the controller's spline
PEAK_FACTOR = 2.0


class ReadyPose(Node):

    def __init__(self, args):
        super().__init__('move_to_ready_pose')
        self.args = args
        side = args.side
        self.joint_names = [f'arm_{side}_{i}_joint' for i in range(1, 8)]
        self.jtc = args.joint_controller or f'arm_{side}_controller'
        self.impedance = args.impedance_controller or f'cartesian_impedance_controller_{side}'
        self.target = np.array(args.joints if args.joints else DEFAULT_READY_RIGHT)

        self.positions = None
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.action = ActionClient(self, FollowJointTrajectory, f'/{self.jtc}/follow_joint_trajectory')
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

    def joint_callback(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in self.joint_names):
            self.positions = np.array([values[name] for name in self.joint_names])

    # --- controller_manager helpers ---------------------------------------------------------------
    def call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{what}: service {client.srv_name} not available')
        future = client.call_async(request)
        deadline = time.time() + 10.0
        while not future.done():
            if time.time() > deadline:
                raise RuntimeError(f'{what}: timed out')
            time.sleep(0.02)
        return future.result()

    def active_controllers(self):
        result = self.call(self.list_client, ListControllers.Request(), 'list_controllers')
        return {c.name for c in result.controller if c.state == 'active'}

    def switch(self, activate, deactivate):
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(sec=5)
        result = self.call(self.switch_client, request, 'switch_controller')
        if not result.ok:
            raise RuntimeError(f'switching controllers failed (activate {activate}, deactivate {deactivate})')

    # --- main sequence ------------------------------------------------------------------------------
    def run(self):
        a = self.args
        self.get_logger().info('Waiting for /joint_states ...')
        deadline = time.time() + 10.0
        while self.positions is None:
            if time.time() > deadline:
                raise RuntimeError(f'no joint states for {self.joint_names}')
            time.sleep(0.05)

        current = self.positions.copy()
        delta = self.target - current
        duration = max(a.min_duration, PEAK_FACTOR * float(np.max(np.abs(delta))) / a.max_joint_speed)

        print('\nJoint            current    target     delta')
        for name, c, t, d in zip(self.joint_names, current, self.target, delta):
            print(f'{name:16s} {c:8.3f} {t:9.3f} {d:9.3f}')
        print(f'\nDuration {duration:.1f} s (largest change {np.max(np.abs(delta)):.2f} rad, '
              f'max joint speed about {a.max_joint_speed} rad/s)')
        print('WARNING: straight joint-space motion, not collision checked. Clear the area, e-stop in reach.')
        if not a.yes and input('Proceed? [y/N] ').strip().lower() != 'y':
            print('Aborted.')
            return 1

        active = self.active_controllers()
        if self.jtc not in active:
            deactivate = [self.impedance] if self.impedance in active else []
            self.get_logger().info(f'Activating {self.jtc}' + (f', deactivating {self.impedance}' if deactivate else ''))
            self.switch([self.jtc], deactivate)

        if not self.action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'action server /{self.jtc}/follow_joint_trajectory not available')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.target.tolist()
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        goal.trajectory.points = [point]

        self.get_logger().info('Moving (Ctrl-C cancels) ...')
        send = self.action.send_goal_async(goal)
        while not send.done():
            time.sleep(0.02)
        handle = send.result()
        if not handle.accepted:
            raise RuntimeError('trajectory goal rejected by the controller')
        result = handle.get_result_async()
        try:
            while not result.done():
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.get_logger().warn('Cancelling the motion')
            handle.cancel_goal_async()
            time.sleep(0.5)
            return 1
        code = result.result().result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'trajectory finished with error code {code} {result.result().result.error_string}')
        self.get_logger().info('Reached the ready pose.')

        if not a.no_switch:
            self.get_logger().info(f'Switching to {self.impedance}')
            self.switch([self.impedance], [self.jtc])
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--side', choices=['right', 'left'], default='right')
    parser.add_argument('--joints', type=float, nargs=7, default=None,
                        help='Target joint angles (7 values); default is the impedance controller nullspace pose.')
    parser.add_argument('--max-joint-speed', type=float, default=0.2,
                        help='Approximate peak joint speed [rad/s]; lower is slower.')
    parser.add_argument('--min-duration', type=float, default=6.0, help='Shortest allowed move [s].')
    parser.add_argument('--joint-controller', default=None)
    parser.add_argument('--impedance-controller', default=None)
    parser.add_argument('--no-switch', action='store_true',
                        help='Leave the joint trajectory controller active at the end.')
    parser.add_argument('--yes', action='store_true', help='Do not ask for confirmation.')
    args, ros_args = parser.parse_known_args()
    if args.side == 'left' and not args.joints:
        parser.error('--joints is required for the left arm (no default ready pose)')

    rclpy.init(args=ros_args)
    node = ReadyPose(args)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    code = 1
    try:
        code = node.run()
    except RuntimeError as ex:
        node.get_logger().error(str(ex))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
