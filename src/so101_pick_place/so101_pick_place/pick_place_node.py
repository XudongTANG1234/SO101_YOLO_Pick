"""
SO-101 Pick & Place Node
/detected_object_pose → MoveIt2 pick 시퀀스 (5-DOF, position-only planning)

pymoveit2의 sync 메서드(plan, execute, move_to_configuration, wait_until_executed)는
내부적으로 rclpy.spin_once()를 호출하여 MultiThreadedExecutor와 충돌한다.
따라서 모든 호출을 async 버전 + time.sleep 대기로 처리한다.
"""

import time
import threading

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from control_msgs.action import ParallelGripperCommand
from pymoveit2 import MoveIt2


# SO-101 pick 자세 (SRDF 'pick' state)
# shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll
PICK_CONFIG = [0.0, 0.0, 0.0, 1.5708, 0.0]


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place')

        self.declare_parameter('pre_grasp_offset', 0.05)
        self.declare_parameter('standoff_distance', 0.05)
        self.declare_parameter('final_approach_distance', 0.0)
        self.declare_parameter('use_dynamic_final_approach', True)
        self.declare_parameter('dynamic_approach_scale', 0.5)
        self.declare_parameter('dynamic_approach_margin', 0.005)
        self.declare_parameter('dynamic_approach_min', 0.01)
        self.declare_parameter('dynamic_approach_max', 0.05)
        self.declare_parameter('use_thickness_for_approach', True)
        self.declare_parameter('gripper_open_ratio', 0.8)
        self.declare_parameter('use_dynamic_gripper_open', True)
        self.declare_parameter('gripper_width_margin_m', 0.01)
        self.declare_parameter('gripper_width_reference_m', 0.09)
        self.declare_parameter('dynamic_gripper_open_min', 0.35)
        self.declare_parameter('dynamic_gripper_open_max', 1.0)
        self.declare_parameter('final_insertion_step_distance', 0.005)
        self.declare_parameter('gripper_open_delay_sec', 0.25)
        self.declare_parameter('max_velocity', 0.3)
        self.declare_parameter('max_acceleration', 0.3)

        self.pre_grasp_offset = self.get_parameter('pre_grasp_offset').value
        self.standoff_distance = self.get_parameter('standoff_distance').value
        self.final_approach_distance = self.get_parameter('final_approach_distance').value
        self.use_dynamic_final_approach = self.get_parameter('use_dynamic_final_approach').value
        self.dynamic_approach_scale = self.get_parameter('dynamic_approach_scale').value
        self.dynamic_approach_margin = self.get_parameter('dynamic_approach_margin').value
        self.dynamic_approach_min = self.get_parameter('dynamic_approach_min').value
        self.dynamic_approach_max = self.get_parameter('dynamic_approach_max').value
        self.use_thickness_for_approach = self.get_parameter('use_thickness_for_approach').value
        self.gripper_open_ratio = self.get_parameter('gripper_open_ratio').value
        self.use_dynamic_gripper_open = self.get_parameter('use_dynamic_gripper_open').value
        self.gripper_width_margin_m = self.get_parameter('gripper_width_margin_m').value
        self.gripper_width_reference_m = self.get_parameter('gripper_width_reference_m').value
        self.dynamic_gripper_open_min = self.get_parameter('dynamic_gripper_open_min').value
        self.dynamic_gripper_open_max = self.get_parameter('dynamic_gripper_open_max').value
        self.final_insertion_step_distance = self.get_parameter('final_insertion_step_distance').value
        self.gripper_open_delay_sec = self.get_parameter('gripper_open_delay_sec').value

        callback_group = ReentrantCallbackGroup()

        # MoveIt2 — manipulator (5-DOF)
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'shoulder_pan',
                'shoulder_lift',
                'elbow_flex',
                'wrist_flex',
                'wrist_roll',
            ],
            base_link_name='base_link',
            end_effector_name='gripper_frame_link',
            group_name='manipulator',
            callback_group=callback_group,
            use_move_group_action=False,
        )
        self.moveit2.max_velocity = self.get_parameter('max_velocity').value
        self.moveit2.max_acceleration = self.get_parameter('max_acceleration').value
        self.moveit2.allowed_planning_time = 10.0
        self.moveit2.num_planning_attempts = 10

        # Gripper — controller action에 직접 연결
        self.gripper_action_client = ActionClient(
            self,
            ParallelGripperCommand,
            '/follower/gripper_controller/gripper_cmd',
            callback_group=callback_group,
        )
        self.latest_pose = None
        self.latest_grip_angle = 0.0
        self.latest_normal = None  # (x, y, z) 법선벡터
        self.latest_object_size = None
        self.latest_object_thickness = None
        self.current_wrist_roll = 0.0
        self.latest_joint_state_msg = None
        self.active_target = None
        self._last_joint_goal = None

        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self._pose_cb,
            10,
            callback_group=callback_group,
        )

        self.sub_joint_states = self.create_subscription(
            JointState,
            'joint_states',
            self._joint_states_cb,
            10,
            callback_group=callback_group,
        )

        self.sub_grip = self.create_subscription(
            Float64,
            '/detected_grip_angle',
            self._grip_cb,
            10,
            callback_group=callback_group,
        )

        self.sub_normal = self.create_subscription(
            Vector3Stamped,
            '/detected_surface_normal',
            self._normal_cb,
            10,
            callback_group=callback_group,
        )

        self.sub_size = self.create_subscription(
            Float64,
            '/detected_object_size',
            self._size_cb,
            10,
            callback_group=callback_group,
        )

        self.sub_thickness = self.create_subscription(
            Float64,
            '/detected_object_thickness',
            self._thickness_cb,
            10,
            callback_group=callback_group,
        )

        self.sub_cmd = self.create_subscription(
            String,
            '/pick_place_cmd',
            self._cmd_cb,
            10,
            callback_group=callback_group,
        )

        self.is_busy = False

        self.get_logger().info('SO-101 Pick & Place ready.')
        self.get_logger().info('  "plan"    -> plan to detected object (no execute)')
        self.get_logger().info('  "execute" -> execute last plan')
        self.get_logger().info('  "go"      -> move to detected object')
        self.get_logger().info('  "pick"    -> full pick sequence')
        self.get_logger().info('  "all"     -> open gripper while moving directly to computed final pose')
        self.get_logger().info('  "home"    -> return to pick pose')

    def _pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg
        self.get_logger().info(
            f'Object at ({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})',
            throttle_duration_sec=2.0,
        )

    def _joint_states_cb(self, msg: JointState):
        self.latest_joint_state_msg = msg
        if 'wrist_roll' in msg.name:
            idx = msg.name.index('wrist_roll')
            self.current_wrist_roll = msg.position[idx]

    def _grip_cb(self, msg: Float64):
        self.latest_grip_angle = msg.data

    def _normal_cb(self, msg: Vector3Stamped):
        self.latest_normal = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _size_cb(self, msg: Float64):
        self.latest_object_size = msg.data

    def _thickness_cb(self, msg: Float64):
        self.latest_object_thickness = msg.data

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Command: "{cmd}"')

        if self.is_busy:
            self.get_logger().warn('Busy, ignoring command')
            return

        self.is_busy = True
        try:
            if cmd == 'plan':
                if not self._capture_target_snapshot():
                    return
                self._plan_to_target()
            elif cmd == 'execute':
                self._execute_last_plan()
            elif cmd == 'go':
                if not self._capture_target_snapshot():
                    return
                self._move_to_target()
            elif cmd == 'pick':
                if not self._capture_target_snapshot():
                    return
                self._execute_pick()
            elif cmd == 'all':
                if not self._capture_target_snapshot():
                    return
                self._execute_all()
            elif cmd == 'home':
                self._go_home()
            else:
                self.get_logger().warn(f'Unknown command: {cmd}')
        except Exception as e:
            self.get_logger().error(f'Command failed: {e}')
        finally:
            self.is_busy = False
            self.get_logger().info('Ready.')

    def _capture_target_snapshot(self):
        if self.latest_pose is None:
            self.get_logger().warn('No object detected yet')
            return False

        self.active_target = {
            'pose': self.latest_pose,
            'grip_angle': self.latest_grip_angle,
            'normal': None if self.latest_normal is None else np.array(self.latest_normal, copy=True),
            'object_size': self.latest_object_size,
            'object_thickness': self.latest_object_thickness,
            'timestamp': self.latest_pose.header.stamp,
        }
        p = self.active_target['pose'].pose.position
        frame = self.active_target['pose'].header.frame_id
        size_str = 'unknown' if self.active_target['object_size'] is None else f"{self.active_target['object_size']:.3f}m"
        thickness_str = (
            'unknown'
            if self.active_target['object_thickness'] is None
            else f"{self.active_target['object_thickness']:.3f}m"
        )
        self.get_logger().info(
            f'Target snapshot captured at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) in {frame}, '
            f'size={size_str}, thickness={thickness_str}'
        )
        return True

    def _get_target(self):
        if self.active_target is None:
            raise RuntimeError('No active target snapshot')
        p = self.active_target['pose'].pose.position
        o = self.active_target['pose'].pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    @staticmethod
    def _quat_to_approach_axis(quat_xyzw):
        x, y, z, w = quat_xyzw
        approach = np.array([
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ])
        norm = np.linalg.norm(approach)
        if norm < 1e-6:
            return np.array([0.0, 0.0, 1.0])
        return approach / norm

    @staticmethod
    def _normalize_symmetric_delta(delta):
        return (delta + np.pi / 2.0) % np.pi - np.pi / 2.0

    def _get_snapshot_approach_axis(self, quat_xyzw):
        if self.active_target is not None and self.active_target['normal'] is not None:
            normal = self.active_target['normal']
            norm = np.linalg.norm(normal)
            if norm > 1e-6:
                return -normal / norm
        return self._quat_to_approach_axis(quat_xyzw)

    @staticmethod
    def _offset_point(point_xyz, direction_xyz, distance):
        point = np.array(point_xyz, dtype=np.float64)
        direction = np.array(direction_xyz, dtype=np.float64)
        return tuple(point + direction * distance)

    @staticmethod
    def _points_close(point_a, point_b, tolerance=1e-4):
        return np.linalg.norm(np.array(point_a, dtype=np.float64) - np.array(point_b, dtype=np.float64)) <= tolerance

    def _move_gripper_to_ratio(self, open_ratio):
        target = self._gripper_target_from_ratio(open_ratio)
        self.get_logger().info(f'Gripper move: ratio={open_ratio:.2f}, target={target:.2f}')
        self._send_gripper_target(target, wait=True)
        return target

    @staticmethod
    def _gripper_target_from_ratio(open_ratio):
        open_ratio = float(np.clip(open_ratio, 0.0, 1.0))
        open_val = 1.5
        closed_val = -0.16
        return open_ratio * open_val + (1.0 - open_ratio) * closed_val

    def _command_gripper_to_ratio(self, open_ratio):
        target = self._gripper_target_from_ratio(open_ratio)
        self.get_logger().info(f'Gripper command: ratio={open_ratio:.2f}, target={target:.2f}')
        self._send_gripper_target(target, wait=False)
        return target

    def _send_gripper_target(self, target, wait=False, timeout_sec=5.0):
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Gripper action server not available.')
            return False

        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['gripper']
        goal.command.position = [float(target)]
        goal.command.velocity = []
        goal.command.effort = []

        send_future = self.gripper_action_client.send_goal_async(goal)
        start_time = time.time()
        while not send_future.done():
            time.sleep(0.01)
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn('Gripper goal send timeout.')
                return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Gripper goal was rejected.')
            return False

        if not wait:
            return True

        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done():
            time.sleep(0.01)
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn('Gripper result timeout.')
                return False

        result = result_future.result()
        if result is None:
            self.get_logger().warn('Gripper result was empty.')
            return False
        return True

    def _command_gripper_to_ratio_direct(self, open_ratio, wait=False):
        target = self._gripper_target_from_ratio(open_ratio)
        self.get_logger().info(f'Direct gripper command: ratio={open_ratio:.2f}, target={target:.2f}')
        return self._send_gripper_target(target, wait=wait)

    def _command_gripper_to_ratio_delayed(self, open_ratio, delay_sec, direct=False):
        def _worker():
            time.sleep(max(0.0, float(delay_sec)))
            if direct:
                self._command_gripper_to_ratio_direct(open_ratio, wait=False)
            else:
                self._command_gripper_to_ratio(open_ratio)

        threading.Thread(target=_worker, daemon=True).start()

    def _get_gripper_open_ratio(self):
        if not self.use_dynamic_gripper_open or self.active_target is None:
            return float(self.gripper_open_ratio)

        object_width = self.active_target['object_size']
        if object_width is None:
            return float(self.gripper_open_ratio)

        reference_width = max(1e-3, float(self.gripper_width_reference_m))
        desired_width = float(object_width) + float(self.gripper_width_margin_m)
        open_ratio = desired_width / reference_width
        open_ratio = float(np.clip(
            open_ratio,
            self.dynamic_gripper_open_min,
            self.dynamic_gripper_open_max,
        ))
        self.get_logger().info(
            f'Dynamic gripper open: object_width={object_width:.3f}m '
            f'desired={desired_width:.3f}m ratio={open_ratio:.2f}'
        )
        return open_ratio

    def _get_final_approach_distance(self):
        if not self.use_dynamic_final_approach:
            return self.final_approach_distance

        if self.active_target is None:
            return self.final_approach_distance

        object_measure = None
        if self.use_thickness_for_approach and self.active_target['object_thickness'] is not None:
            object_measure = self.active_target['object_thickness']
        elif self.active_target['object_size'] is not None:
            object_measure = self.active_target['object_size']

        if object_measure is None:
            return self.final_approach_distance

        dynamic_distance = (
            object_measure * self.dynamic_approach_scale
            + self.dynamic_approach_margin
        )
        dynamic_distance = float(np.clip(
            dynamic_distance,
            self.dynamic_approach_min,
            self.dynamic_approach_max,
        ))
        return dynamic_distance

    # ─── async helpers (rclpy.spin_once 사용 안 함) ───

    def _solve_ik(self, x, y, z, quat_xyzw=None, apply_grip_angle=True):
        """IK를 먼저 풀어서 joint configuration을 얻는다.
        apply_grip_angle=True이면 wrist_roll을 현재 상태 대비 필요한 delta만큼 보정한다."""
        if quat_xyzw is None:
            quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        future = self.moveit2.compute_ik_async(
            position=[x, y, z],
            quat_xyzw=quat_xyzw,
        )
        if future is None:
            self.get_logger().warn('IK service not available')
            return None
        ik_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - ik_start > 5.0:
                self.get_logger().warn('IK timeout!')
                return None
        joint_state = self.moveit2.get_compute_ik_result(future)
        if joint_state is None:
            self.get_logger().warn(f'IK failed for ({x:.3f}, {y:.3f}, {z:.3f})')
            return None
        joint_positions = []
        for name in ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']:
            if name in joint_state.name:
                idx = joint_state.name.index(name)
                joint_positions.append(joint_state.position[idx])
            else:
                self.get_logger().warn(f'Joint {name} not in IK result')
                return None
        # grip angle → wrist_roll (마지막 조인트) 덮어쓰기
        grip_angle = 0.0
        if self.active_target is not None:
            grip_angle = self.active_target['grip_angle']

        if apply_grip_angle and grip_angle != 0.0:
            current_wrist_roll = self.current_wrist_roll
            delta = self._normalize_symmetric_delta(grip_angle - current_wrist_roll)
            target_wrist_roll = current_wrist_roll + delta
            joint_positions[4] = target_wrist_roll
            self.get_logger().info(
                f'IK solved: {[f"{v:.3f}" for v in joint_positions]} '
                f'(current={np.degrees(current_wrist_roll):.0f}°, '
                f'target={np.degrees(grip_angle):.0f}°, '
                f'delta={np.degrees(delta):.0f}°)'
            )
        else:
            self.get_logger().info(f'IK solved: {[f"{v:.3f}" for v in joint_positions]}')
        return joint_positions

    def _plan_joints_async(self, joint_positions):
        """plan_async → time.sleep 대기 → trajectory 반환."""
        future = self.moveit2.plan_async(joint_positions=joint_positions)
        if future is None:
            return None
        while not future.done():
            time.sleep(0.01)
        return self.moveit2.get_trajectory(future)

    def _plan_cartesian_pose_async(self, x, y, z, quat_xyzw, max_step=0.0025, fraction_threshold=0.9):
        """Cartesian path planning without blocking spin_once in pymoveit2 plan()."""
        start_joint_state = self.latest_joint_state_msg
        if start_joint_state is None:
            self.get_logger().warn('No joint state available for Cartesian planning.')
            return None

        future = self.moveit2.plan_async(
            position=[x, y, z],
            quat_xyzw=quat_xyzw,
            frame_id='base_link',
            start_joint_state=start_joint_state,
            cartesian=True,
            max_step=max_step,
        )
        if future is None:
            self.get_logger().warn('Cartesian planning service not available.')
            return None

        start_time = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start_time > 5.0:
                self.get_logger().warn('Cartesian planning timeout!')
                return None

        return self.moveit2.get_trajectory(
            future,
            cartesian=True,
            cartesian_fraction_threshold=fraction_threshold,
        )

    def _execute_stepwise_insertion(self, start_point, approach_axis, distance, quat_xyzw):
        if distance <= 0.0:
            return True

        step_distance = max(1e-3, float(self.final_insertion_step_distance))
        num_steps = max(1, int(np.ceil(distance / step_distance)))
        self.get_logger().info(
            f'Final insertion will use {num_steps} step(s), step_distance~{distance / num_steps:.3f}m'
        )

        start = np.array(start_point, dtype=np.float64)
        direction = np.array(approach_axis, dtype=np.float64)
        direction_norm = np.linalg.norm(direction)
        if direction_norm < 1e-6:
            self.get_logger().warn('Invalid approach axis for final insertion.')
            return False
        direction = direction / direction_norm

        for step_index in range(1, num_steps + 1):
            waypoint = tuple(start + direction * (distance * step_index / num_steps))
            self.get_logger().info(
                f'  insertion step {step_index}/{num_steps}: '
                f'({waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f})'
            )
            if not self._move_to_position(*waypoint, quat_xyzw):
                self.get_logger().warn(f'Insertion step {step_index}/{num_steps} failed.')
                return False
        return True

    def _execute_and_wait(self, trajectory):
        """execute (async send) → __is_executing 플래그로 대기."""
        self.moveit2.execute(trajectory)
        # execute()는 _send_goal_async_execute_trajectory만 호출 (spin 안 함)
        # wait는 내부 플래그로 직접 대기
        timeout = 30.0
        start = time.time()
        while (self.moveit2._MoveIt2__is_motion_requested or
               self.moveit2._MoveIt2__is_executing):
            time.sleep(0.05)
            if time.time() - start > timeout:
                self.get_logger().warn('Execution timeout!')
                return False
        return self.moveit2.motion_suceeded

    def _move_joints(self, joint_positions):
        """IK 결과 joint config → plan → execute (모두 async)."""
        trajectory = self._plan_joints_async(joint_positions)
        if trajectory is None:
            self.get_logger().warn('Planning failed (OMPL).')
            return False
        self.get_logger().info('Plan found, executing...')
        return self._execute_and_wait(trajectory)

    # ─── commands ───

    def _move_to_position(self, x, y, z, quat_xyzw=None):
        """IK → joint configuration → plan & execute."""
        self.get_logger().info(f'Moving to ({x:.3f}, {y:.3f}, {z:.3f})...')
        joint_positions = self._solve_ik(x, y, z, quat_xyzw)
        if joint_positions is None:
            return False
        ok = self._move_joints(joint_positions)
        if ok:
            self.get_logger().info('Arrived.')
        else:
            self.get_logger().warn('Motion may have failed.')
        return ok

    def _plan_to_position(self, x, y, z, quat_xyzw=None):
        """Plan only (no execute). IK → joint config → plan."""
        self.get_logger().info(f'Planning to ({x:.3f}, {y:.3f}, {z:.3f})...')
        joint_positions = self._solve_ik(x, y, z, quat_xyzw)
        if joint_positions is None:
            self.get_logger().warn('Planning failed (IK).')
            return False
        trajectory = self._plan_joints_async(joint_positions)
        if trajectory is not None:
            self.get_logger().info('Plan found. Check RViz. Send "execute" to run.')
            self._last_joint_goal = joint_positions
        else:
            self.get_logger().warn('Planning failed (OMPL).')
        return trajectory is not None

    def _plan_to_target(self):
        (x, y, z), quat = self._get_target()
        z += self.pre_grasp_offset
        self._plan_to_position(x, y, z, quat)

    def _execute_last_plan(self):
        if self._last_joint_goal is None:
            self.get_logger().warn('No plan to execute. Run "plan" first.')
            return
        self.get_logger().info('Executing last plan...')
        ok = self._move_joints(self._last_joint_goal)
        self._last_joint_goal = None
        if ok:
            self.get_logger().info('Executed.')
        else:
            self.get_logger().warn('Execution may have failed.')

    def _move_to_target(self):
        """고정된 snapshot 기준으로 standoff 후 목표까지 접근."""
        (x, y, z), quat = self._get_target()
        approach_axis = self._get_snapshot_approach_axis(quat)
        standoff = self._offset_point((x, y, z), -approach_axis, self.standoff_distance)

        self.get_logger().info(
            f'Go sequence: standoff={self.standoff_distance:.3f}m '
            f'approach_axis=({approach_axis[0]:.3f}, {approach_axis[1]:.3f}, {approach_axis[2]:.3f})'
        )
        self.get_logger().info(
            f'Step 1/2: Standoff ({standoff[0]:.3f}, {standoff[1]:.3f}, {standoff[2]:.3f})'
        )
        if not self._move_to_position(*standoff, quat):
            self.get_logger().warn('Standoff failed.')
            return

        self.get_logger().info(f'Step 2/2: Target ({x:.3f}, {y:.3f}, {z:.3f})')
        self._move_to_position(x, y, z, quat)

    def _execute_pick(self):
        """Pick sequence: standoff -> open -> target -> fixed insertion -> close."""
        (x, y, z), quat = self._get_target()
        approach_axis = self._get_snapshot_approach_axis(quat)
        standoff = self._offset_point((x, y, z), -approach_axis, self.standoff_distance)
        final_approach_distance = self._get_final_approach_distance()
        insertion = self._offset_point((x, y, z), approach_axis, final_approach_distance)

        self.get_logger().info('=== Pick Start ===')
        self.get_logger().info(
            f'Fixed snapshot target: pose=({x:.3f}, {y:.3f}, {z:.3f}) '
            f'standoff={self.standoff_distance:.3f}m insertion={final_approach_distance:.3f}m'
        )
        self.get_logger().info(
            f'Approach axis=({approach_axis[0]:.3f}, {approach_axis[1]:.3f}, {approach_axis[2]:.3f})'
        )

        # Step 1: 그리퍼 열기
        open_ratio = self._get_gripper_open_ratio()
        self.get_logger().info(f'Step 1/5: Open gripper ({open_ratio:.2f})')
        self._move_gripper_to_ratio(open_ratio)

        # Step 2: standoff 위치로 이동
        self.get_logger().info(
            f'Step 2/5: Standoff ({standoff[0]:.3f}, {standoff[1]:.3f}, {standoff[2]:.3f})'
        )
        if not self._move_to_position(*standoff, quat):
            self.get_logger().error('Standoff failed, aborting.')
            return

        # Step 3: snapshot target까지 접근
        if self._points_close(standoff, (x, y, z)):
            self.get_logger().info('Step 3/5: Target skipped (already at target pose)')
        else:
            self.get_logger().info(f'Step 3/5: Target ({x:.3f}, {y:.3f}, {z:.3f})')
            if not self._move_to_position(x, y, z, quat):
                self.get_logger().warn('Target approach failed.')
                return

        # Step 4: 고정 quaternion 접근축으로 최종 삽입
        if final_approach_distance > 0.0:
            self.get_logger().info(
                f'Step 4/5: Final insertion ({insertion[0]:.3f}, {insertion[1]:.3f}, {insertion[2]:.3f})'
            )
            if not self._execute_stepwise_insertion((x, y, z), approach_axis, final_approach_distance, quat):
                self.get_logger().warn('Final insertion failed.')
                return
        else:
            self.get_logger().info('Step 4/5: Final insertion skipped (distance=0.0)')

        # Step 5: 그리퍼 닫기
        self.get_logger().info('Step 5/5: Close gripper')
        self._move_gripper_to_ratio(0.0)

        self.get_logger().info('=== Pick Complete ===')

    def _execute_all(self):
        (x, y, z), quat = self._get_target()
        approach_axis = self._get_snapshot_approach_axis(quat)
        final_approach_distance = self._get_final_approach_distance()
        insertion = self._offset_point((x, y, z), approach_axis, final_approach_distance)
        open_ratio = self._get_gripper_open_ratio()

        self.get_logger().info('=== All Start ===')
        self.get_logger().info(
            f'snapshot target=({x:.3f}, {y:.3f}, {z:.3f}) '
            f'final_pose=({insertion[0]:.3f}, {insertion[1]:.3f}, {insertion[2]:.3f}) '
            f'distance={final_approach_distance:.3f}m open_ratio={open_ratio:.2f}'
        )

        self.get_logger().info(
            f'Arm moves first, gripper open command will follow after {self.gripper_open_delay_sec:.2f}s'
        )
        self._command_gripper_to_ratio_delayed(
            open_ratio,
            self.gripper_open_delay_sec,
            direct=True,
        )

        if self._move_to_position(*insertion, quat):
            self.get_logger().info('Step 2/2: Close gripper')
            self._command_gripper_to_ratio_direct(0.0, wait=True)
            self.get_logger().info('=== All Complete ===')
        else:
            self.get_logger().warn('All move failed.')

    def _go_home(self):
        self.get_logger().info('Returning to pick pose...')
        ok = self._move_joints(PICK_CONFIG)
        if ok:
            self.get_logger().info('Home.')
        else:
            self.get_logger().warn('Home motion may have failed.')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
