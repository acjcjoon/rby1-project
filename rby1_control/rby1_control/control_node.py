"""Single robot-facing ROS 2 control node.

Supported paths:
- mobile base velocity through geometry_msgs/Twist
- power / servo / stream control through StateOnOff services
- robot state monitoring
- joint state monitoring
- joint position commands through Rby1JointCommand
- Cartesian pose monitoring through GetCartesianPose
- Cartesian position commands through Rby1CartesianCommand
- motion cancellation through cancel_control
"""

from __future__ import annotations

from collections import deque
import math
import threading
import time
from typing import Deque, Dict, List, Optional, Tuple

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from .backend_contract import (
    BackendSnapshot,
    TaskBackendState,
    TaskCommandState,
    TaskCommandStatus,
    VelocityCommand,
)
from .control_transport import ControlTransport
from .gripper_controller import GripperCommandError, GripperController
from .manipulator_controller import ManipulatorController
from .mobile_base_controller import MobileBaseController
from .robot_session import RobotSession
from .state_manager import StateManager
from .command_model import CommandKind, TaskCommand

try:
    from rby1_msgs.action import (
        Rby1CartesianCommand,
        Rby1JointCommand,
    )
    from rby1_msgs.msg import (
        CartesianCommand,
        JointCommand,
        RobotState,
        ToolFlangeState,
    )
    from rby1_msgs.srv import (
        ControlManagerCommand,
        GetCartesianPose,
        StateOnOff,
    )

    RBY1_MSGS_AVAILABLE = True
except ImportError:
    Rby1CartesianCommand = None  # type: ignore[assignment]
    Rby1JointCommand = None  # type: ignore[assignment]
    CartesianCommand = None  # type: ignore[assignment]
    JointCommand = None  # type: ignore[assignment]
    RobotState = None  # type: ignore[assignment]
    ToolFlangeState = None  # type: ignore[assignment]
    ControlManagerCommand = None  # type: ignore[assignment]
    GetCartesianPose = None  # type: ignore[assignment]
    StateOnOff = None  # type: ignore[assignment]
    RBY1_MSGS_AVAILABLE = False

# RB-Y1 M v1.3 joint position limits from model_v1_3.urdf.
# Unit: rad
JOINT_LIMITS_RAD = {
    "torso": (
        (-0.261799388, 0.261799388),
        (-0.523598776, 1.570796327),
        (-2.617993878, 1.570796327),
        (-0.785398163, 1.570796327),
        (-0.523598776, 0.523598776),
        (-2.35619449, 2.35619449),
    ),
    "right_arm": (
        (-3.141592654, 3.141592654),
        (-3.141592654, 0.017453293),
        (-3.141592654, 3.141592654),
        (-2.617993878, 0.017453293),
        (-3.141592654, 3.141592654),
        (-0.8726646260, 0.8726646260),
        (-1.5707963268, 1.5707963268),
    ),
    "left_arm": (
        (-3.141592654, 3.141592654),
        (-0.017453293, 3.141592654),
        (-3.141592654, 3.141592654),
        (-2.617993878, 0.017453293),
        (-3.141592654, 3.141592654),
        (-0.8726646260, 0.8726646260),
        (-1.5707963268, 1.5707963268),
    ),
    "head": (
        (-1.57, 1.57),
        (-1.57, 1.57),
    ),
}

JOINT_MAX_VELOCITY_RAD = {
    'torso': (2.09439510, 2.09439510, 2.09439510, math.pi, math.pi, math.pi),
    'right_arm': (
        math.pi, math.pi, math.pi, math.pi,
        2 * math.pi, 2 * math.pi, 2.094395102,
    ),
    'left_arm': (
        math.pi, math.pi, math.pi, math.pi,
        2 * math.pi, 2 * math.pi, 2.094395102,
    ),
    'head': (3.14, 3.14),
}
JOINT_MAX_ACCELERATION_RAD = {
    'torso': (5.0,) * 6,
    'right_arm': (10.0,) * 7,
    'left_arm': (10.0,) * 7,
    # The model does not provide verified head acceleration limits.
    'head': (None, None),
}

ROBOT_STATE_MAX_AGE_SEC = 1.0
MANIPULATOR_STATE_MAX_AGE_SEC = 1.0
CONTROL_MANAGER_FAULT_STATES = (4, 5)

TASK_CARTESIAN_STOP_POSITION_ERROR_M = 1e-3
TASK_CARTESIAN_STOP_ORIENTATION_ERROR_RAD = 1e-2

# The physical gripper electronics are a 12 V device.  Treat voltage feedback
# within 0.5 V of nominal as confirmed, but never expose a selectable voltage
# through the UI/backend API.
GRIPPER_POWER_VOLTAGE = 12.0
GRIPPER_POWER_TOLERANCE = 0.5


class RBY1ControlNode(Node):
    """Own every robot-facing interface in one ROS node."""

    def __init__(self, node_name: str = 'rby1_control') -> None:
        super().__init__(str(node_name), namespace='rby1')

        # ROS topic and service names.
        self.declare_parameter('cmd_raw_topic', 'cmd_raw')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('robot_state_topic', 'robot_state')
        self.declare_parameter('robot_address', '')
        self.declare_parameter('robot_model', 'm')
        self.declare_parameter('power_pattern', '.*')
        self.declare_parameter('servo_pattern', '.*')
        self.declare_parameter('power_servo_poll_period_sec', 0.5)
        self.declare_parameter(
            'power_servo_feedback_timeout_sec',
            1.5,
        )
        self.declare_parameter('robot_power_service', 'robot_power')
        self.declare_parameter('robot_servo_service', 'robot_servo')
        self.declare_parameter(
            'stream_control_service',
            'stream_control',
        )
        self.declare_parameter(
            'control_manager_service',
            'control_manager_command',
        )

        # Manipulation interfaces exposed by rby1_driver.
        self.declare_parameter('joint_action', 'robot_joint')
        self.declare_parameter(
            'cartesian_action',
            'robot_cartesian',
        )
        self.declare_parameter(
            'cartesian_pose_service',
            'get_cartesian_pose',
        )
        self.declare_parameter(
            'cancel_control_service',
            'cancel_control',
        )
        self.declare_parameter(
            'right_arm_joint_state_topic',
            'joint_states/right_arm',
        )
        self.declare_parameter(
            'left_arm_joint_state_topic',
            'joint_states/left_arm',
        )
        self.declare_parameter(
            'torso_joint_state_topic',
            'joint_states/torso',
        )
        self.declare_parameter(
            'head_joint_state_topic',
            'joint_states/head',
        )

        # Normalized dual-gripper interface exposed by rby1_gripper_driver.
        self.declare_parameter('gripper_command_topic', 'gripper/command')
        self.declare_parameter('gripper_state_topic', 'gripper/state')
        self.declare_parameter('gripper_ready_topic', 'gripper/ready')
        self.declare_parameter('gripper_open_ratio', 0.0)
        self.declare_parameter('gripper_close_ratio', 1.0)
        self.declare_parameter('gripper_state_timeout_sec', 1.0)
        self.declare_parameter('gripper_command_timeout_sec', 5.0)
        self.declare_parameter('gripper_goal_tolerance', 0.05)
        self.declare_parameter('gripper_home_service', 'gripper/home')
        self.declare_parameter(
            'gripper_torque_service',
            'gripper/torque_enable',
        )
        self.declare_parameter(
            'tool_flange_power_service',
            'tool_flange_power',
        )
        self.declare_parameter(
            'tool_flange_right_state_topic',
            'tool_flange/right',
        )
        self.declare_parameter(
            'tool_flange_left_state_topic',
            'tool_flange/left',
        )
        self.declare_parameter('tool_flange_state_timeout_sec', 1.5)

        # Current RB-Y1 M v1.3 Cartesian operator frame.
        self.declare_parameter(
            'right_cartesian_ref_link',
            'base',
        )
        self.declare_parameter(
            'right_cartesian_target_link',
            'ee_right',
        )
        self.declare_parameter(
            'left_cartesian_ref_link',
            'base',
        )
        self.declare_parameter(
            'left_cartesian_target_link',
            'ee_left',
        )

        # Backend behavior.
        self.declare_parameter('use_rby1_services', True)
        self.declare_parameter('publish_rate_hz', 25.0)
        self.declare_parameter('command_timeout_sec', 0.35)
        self.declare_parameter('publish_zero_when_idle', True)
        self.declare_parameter(
            'cartesian_state_period_sec',
            0.25,
        )
        self.declare_parameter(
            'joint_jog_minimum_time_sec',
            1.0,
        )
        self.declare_parameter(
            'cartesian_jog_minimum_time_sec',
            1.0,
        )
        # Conservative limits for manual MOVE TARGET and jog commands.
        # Scenario commands keep their explicit values from task.py.
        self.declare_parameter('manual_joint_velocity_limit', 0.2)
        self.declare_parameter('manual_joint_acceleration_limit', 0.2)
        self.declare_parameter(
            'manual_cartesian_linear_velocity_limit',
            0.05,
        )
        self.declare_parameter(
            'manual_cartesian_angular_velocity_limit',
            0.2,
        )
        self.declare_parameter(
            'manual_cartesian_acceleration_scaling',
            0.2,
        )

        self.cmd_raw_topic = str(
            self.get_parameter('cmd_raw_topic').value
        )
        self.cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )
        self.robot_state_topic = str(
            self.get_parameter('robot_state_topic').value
        )
        self.robot_address = str(
            self.get_parameter('robot_address').value
        ).strip()
        self.robot_model = str(
            self.get_parameter('robot_model').value
        ).strip().lower()
        self.power_pattern = str(
            self.get_parameter('power_pattern').value
        )
        self.servo_pattern = str(
            self.get_parameter('servo_pattern').value
        )
        self.power_servo_poll_period_sec = self._positive_float(
            self.get_parameter('power_servo_poll_period_sec').value,
            fallback=0.5,
        )
        self.power_servo_feedback_timeout_sec = self._positive_float(
            self.get_parameter(
                'power_servo_feedback_timeout_sec'
            ).value,
            fallback=1.5,
        )
        self.robot_power_service = str(
            self.get_parameter('robot_power_service').value
        )
        self.robot_servo_service = str(
            self.get_parameter('robot_servo_service').value
        )
        self.stream_control_service = str(
            self.get_parameter('stream_control_service').value
        )
        self.control_manager_service = str(
            self.get_parameter('control_manager_service').value
        )

        self.joint_action_name = str(
            self.get_parameter('joint_action').value
        )
        self.cartesian_action_name = str(
            self.get_parameter('cartesian_action').value
        )
        self.cartesian_pose_service = str(
            self.get_parameter('cartesian_pose_service').value
        )
        self.cancel_control_service = str(
            self.get_parameter('cancel_control_service').value
        )
        self.joint_state_topics = {
            'right_arm': str(
                self.get_parameter(
                    'right_arm_joint_state_topic'
                ).value
            ),
            'left_arm': str(
                self.get_parameter(
                    'left_arm_joint_state_topic'
                ).value
            ),
            'torso': str(
                self.get_parameter(
                    'torso_joint_state_topic'
                ).value
            ),
            'head': str(
                self.get_parameter(
                    'head_joint_state_topic'
                ).value
            ),
        }
        self.gripper_command_topic = str(
            self.get_parameter('gripper_command_topic').value
        )
        self.gripper_state_topic = str(
            self.get_parameter('gripper_state_topic').value
        )
        self.gripper_ready_topic = str(
            self.get_parameter('gripper_ready_topic').value
        )
        self.gripper_home_service = str(
            self.get_parameter('gripper_home_service').value
        )
        self.gripper_torque_service = str(
            self.get_parameter('gripper_torque_service').value
        )
        self.tool_flange_power_service = str(
            self.get_parameter('tool_flange_power_service').value
        )
        self.tool_flange_state_topics = {
            'right': str(
                self.get_parameter(
                    'tool_flange_right_state_topic'
                ).value
            ),
            'left': str(
                self.get_parameter(
                    'tool_flange_left_state_topic'
                ).value
            ),
        }
        self.tool_flange_state_timeout_sec = self._positive_float(
            self.get_parameter('tool_flange_state_timeout_sec').value,
            fallback=1.5,
        )

        self.cartesian_links = {
            'right_arm': (
                str(
                    self.get_parameter(
                        'right_cartesian_ref_link'
                    ).value
                ),
                str(
                    self.get_parameter(
                        'right_cartesian_target_link'
                    ).value
                ),
            ),
            'left_arm': (
                str(
                    self.get_parameter(
                        'left_cartesian_ref_link'
                    ).value
                ),
                str(
                    self.get_parameter(
                        'left_cartesian_target_link'
                    ).value
                ),
            ),
        }

        requested_services = bool(
            self.get_parameter('use_rby1_services').value
        )

        self.services_enabled = (
            requested_services and RBY1_MSGS_AVAILABLE
        )

        self.publish_rate_hz = self._positive_float(
            self.get_parameter('publish_rate_hz').value,
            fallback=25.0,
        )

        self.command_timeout_sec = self._positive_float(
            self.get_parameter('command_timeout_sec').value,
            fallback=0.35,
        )

        self.publish_zero_when_idle = bool(
            self.get_parameter('publish_zero_when_idle').value
        )

        self.cartesian_state_period_sec = self._positive_float(
            self.get_parameter(
                'cartesian_state_period_sec'
            ).value,
            fallback=0.25,
        )

        self.joint_jog_minimum_time_sec = self._positive_float(
            self.get_parameter(
                'joint_jog_minimum_time_sec'
            ).value,
            fallback=1.0,
        )

        self.cartesian_jog_minimum_time_sec = self._positive_float(
            self.get_parameter(
                'cartesian_jog_minimum_time_sec'
            ).value,
            fallback=1.0,
        )

        self.manual_joint_velocity_limit = self._positive_float(
            self.get_parameter('manual_joint_velocity_limit').value,
            fallback=0.2,
        )
        self.manual_joint_acceleration_limit = self._positive_float(
            self.get_parameter('manual_joint_acceleration_limit').value,
            fallback=0.2,
        )
        self.manual_cartesian_linear_velocity_limit = self._positive_float(
            self.get_parameter(
                'manual_cartesian_linear_velocity_limit'
            ).value,
            fallback=0.05,
        )
        self.manual_cartesian_angular_velocity_limit = self._positive_float(
            self.get_parameter(
                'manual_cartesian_angular_velocity_limit'
            ).value,
            fallback=0.2,
        )
        acceleration_scaling = self._positive_float(
            self.get_parameter(
                'manual_cartesian_acceleration_scaling'
            ).value,
            fallback=0.2,
        )
        self.manual_cartesian_acceleration_scaling = (
            acceleration_scaling if acceleration_scaling <= 1.0 else 0.2
        )

        self.gripper_controller = GripperController(
            open_ratio=float(
                self.get_parameter('gripper_open_ratio').value
            ),
            close_ratio=float(
                self.get_parameter('gripper_close_ratio').value
            ),
            state_timeout_sec=self._positive_float(
                self.get_parameter('gripper_state_timeout_sec').value,
                fallback=1.0,
            ),
            command_timeout_sec=self._positive_float(
                self.get_parameter('gripper_command_timeout_sec').value,
                fallback=5.0,
            ),
            goal_tolerance=self._positive_float(
                self.get_parameter('gripper_goal_tolerance').value,
                fallback=0.05,
            ),
        )
        self._gripper_controller = self.gripper_controller
        self._last_gripper_state_warning_at = -math.inf
        self._tool_flange_voltages: Dict[str, Optional[float]] = {
            'right': None,
            'left': None,
        }
        self._tool_flange_updated_at: Dict[str, Optional[float]] = {
            'right': None,
            'left': None,
        }
        self._gripper_power_off_pending = False
        self._gripper_home_pending = False

        # Shared state and event ownership.
        self._lock = threading.RLock()
        self._events: Deque[Tuple[str, str]] = deque(maxlen=300)
        self.state_manager = StateManager()
        self.robot_session = RobotSession(
            state_manager=self.state_manager,
            robot_address=self.robot_address,
            robot_model=self.robot_model,
            power_pattern=self.power_pattern,
            servo_pattern=self.servo_pattern,
            poll_period_sec=self.power_servo_poll_period_sec,
            on_event=self._push_event,
        )

        # Actual robot state received from /robot_state.
        self.control_state: Optional[int] = None
        self.power_enabled: Optional[bool] = None
        self.servo_enabled: Optional[bool] = None
        self.stream_enabled: Optional[bool] = None
        self.emo_active: Optional[bool] = None
        self.collision_active: Optional[bool] = None

        self._power_feedback_time: Optional[float] = None
        self._servo_feedback_time: Optional[float] = None

        self._robot_state_received = False
        self._robot_state_updated_at: Optional[float] = None

        # ManipulatorController owns feedback caches; aliases keep the command
        # code compact while the state implementation remains independently
        # testable.
        self.manipulator_controller = ManipulatorController()
        self._joint_groups_deg = (
            self.manipulator_controller.joint_groups_deg
        )
        self._joint_updated_at = (
            self.manipulator_controller.joint_updated_at
        )
        self._joint_order_verified = (
            self.manipulator_controller.joint_order_verified
        )
        self._cartesian_state = (
            self.manipulator_controller.cartesian_state
        )
        self._cartesian_updated_at = (
            self.manipulator_controller.cartesian_updated_at
        )
        self._cartesian_quaternion_state = (
            self.manipulator_controller.cartesian_quaternion_state
        )
        self._cartesian_snapshot = (
            self.manipulator_controller.cartesian_snapshot
        )

        self._cartesian_snapshot_pending = {
            'right_arm': False,
            'left_arm': False,
        }

        self._cartesian_request_pending = {
            'right_arm': False,
            'left_arm': False,
        }

        # Keep one manipulation command active at a time.
        self._motion_busy = False
        self._active_motion_kind: Optional[str] = None
        self._active_goal_handle = None
        self._active_task_command_id: Optional[str] = None
        self._task_command_sequence = 0
        self._task_commands: Dict[str, TaskCommandState] = {}
        self._active_gripper_task_id: Optional[str] = None
        self._active_gripper_task_kind: Optional[CommandKind] = None
        self._active_gripper_feedback_marker: Optional[int] = None
        self._active_gripper_settle_deadline: Optional[float] = None
        self._runtime_safety_stop_latched = False

        # Action-level cancellation state for press-and-hold Cartesian jogging.
        self._cancel_motion_on_accept = False
        self._active_cancel_requested = False

        # Prepare Robot state machine.
        self._prepare_stage = 'idle'
        self._prepare_not_before = 0.0
        self._prepare_deadline = 0.0

        # Keep asynchronous service futures alive.
        self._pending_futures: List[object] = []

        self.mobile_base_controller = MobileBaseController(
            self,
            cmd_raw_topic=self.cmd_raw_topic,
            cmd_vel_topic=self.cmd_vel_topic,
            command_timeout_sec=self.command_timeout_sec,
            publish_rate_hz=self.publish_rate_hz,
            publish_zero_when_idle=self.publish_zero_when_idle,
            motion_guard=self._base_motion_safety_reason,
            on_event=self._push_event,
        )
        self.cmd_vel_pub = self.mobile_base_controller.publisher

        self.gripper_command_pub = self.create_publisher(
            Float64MultiArray,
            self.gripper_command_topic,
            10,
        )
        self.gripper_state_sub = self.create_subscription(
            Float64MultiArray,
            self.gripper_state_topic,
            self._gripper_state_callback,
            10,
        )
        gripper_ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.gripper_ready_sub = self.create_subscription(
            Bool,
            self.gripper_ready_topic,
            self._gripper_ready_callback,
            gripper_ready_qos,
        )

        # RB-Y1 service clients and subscriber.
        self.power_client = None
        self.servo_client = None
        self.stream_client = None
        self.control_manager_client = None
        self.tool_flange_power_client = None
        self.state_sub = None
        self.tool_flange_state_subs = []

        # These services belong to the standalone gripper driver and use
        # standard ROS interfaces, so keep them available even if rby1_msgs is
        # missing and the main backend falls back to cmd_vel-only mode.
        self.gripper_home_client = self.create_client(
            Trigger,
            self.gripper_home_service,
        )
        self.gripper_torque_client = self.create_client(
            SetBool,
            self.gripper_torque_service,
        )

        self.joint_action_client = None
        self.cartesian_action_client = None
        self.cartesian_pose_client = None
        self.cancel_control_client = None
        self.joint_state_subs = []

        if self.services_enabled:
            assert StateOnOff is not None
            assert ControlManagerCommand is not None
            assert RobotState is not None
            assert GetCartesianPose is not None
            assert Rby1JointCommand is not None
            assert Rby1CartesianCommand is not None

            self.power_client = self.create_client(
                StateOnOff,
                self.robot_power_service,
            )

            self.servo_client = self.create_client(
                StateOnOff,
                self.robot_servo_service,
            )

            self.stream_client = self.create_client(
                StateOnOff,
                self.stream_control_service,
            )

            self.control_manager_client = self.create_client(
                ControlManagerCommand,
                self.control_manager_service,
            )

            self.tool_flange_power_client = self.create_client(
                StateOnOff,
                self.tool_flange_power_service,
            )

            self.state_sub = self.create_subscription(
                RobotState,
                self.robot_state_topic,
                self._state_callback,
                10,
            )

            assert ToolFlangeState is not None
            for side, topic in self.tool_flange_state_topics.items():
                subscription = self.create_subscription(
                    ToolFlangeState,
                    topic,
                    lambda msg, selected=side:
                    self._tool_flange_state_callback(selected, msg),
                    10,
                )
                self.tool_flange_state_subs.append(subscription)

            self.cartesian_pose_client = self.create_client(
                GetCartesianPose,
                self.cartesian_pose_service,
            )

            self.cancel_control_client = self.create_client(
                Trigger,
                self.cancel_control_service,
            )
            self.joint_action_client = ActionClient(
                self,
                Rby1JointCommand,
                self.joint_action_name,
            )

            self.cartesian_action_client = ActionClient(
                self,
                Rby1CartesianCommand,
                self.cartesian_action_name,
            )

            for group, topic in self.joint_state_topics.items():
                subscription = self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, group_name=group:
                    self._joint_state_callback(
                        group_name,
                        msg,
                    ),
                    10,
                )
                self.joint_state_subs.append(subscription)

        elif requested_services:
            self._push_event(
                'warning',
                'rby1_msgs를 불러오지 못해 '
                'cmd_vel 전용 모드로 시작합니다.',
            )

        self.publish_timer = self.mobile_base_controller.timer
        self.robot_session_timer = self.create_timer(
            0.05,
            self._poll_robot_session,
        )

        # Power -> Servo preparation state machine.
        self.operation_timer = self.create_timer(
            0.05,
            self._process_prepare_operation,
        )

        # Poll Cartesian pose asynchronously.  Joint state arrives by topic.
        self.cartesian_state_timer = self.create_timer(
            self.cartesian_state_period_sec,
            self._poll_cartesian_state,
        )
        self.motion_safety_timer = self.create_timer(
            0.1,
            self._enforce_runtime_motion_safety,
        )
        self.gripper_status_timer = self.create_timer(
            0.1,
            self._check_gripper_command,
        )
        self.transport = ControlTransport(self)

        self._push_event(
            'info',
            f'ROS node started: '
            f'namespace={self.get_namespace()}, '
            f'cmd_raw={self.mobile_base_controller.cmd_raw_topic}, '
            f'cmd_vel={self.mobile_base_controller.cmd_vel_topic}',
        )

    @staticmethod
    def _positive_float(
        value: object,
        fallback: float,
    ) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return fallback

        if not math.isfinite(result) or result <= 0.0:
            return fallback

        return result

    def _push_event(
        self,
        level: str,
        message: str,
    ) -> None:
        timestamp = time.strftime('%H:%M:%S')

        with self._lock:
            self._events.append(
                (level, f'[{timestamp}] {message}')
            )

        logger = self.get_logger()

        if level == 'error':
            logger.error(message)
        elif level == 'warning':
            logger.warning(message)
        else:
            logger.info(message)

    def drain_events(self) -> List[Tuple[str, str]]:
        with self._lock:
            items = list(self._events)
            self._events.clear()

        return items

    def _poll_robot_session(self) -> None:
        """Advance async SDK work and mirror its latest state for commands."""

        self.robot_session.poll()
        status = self.robot_session.snapshot()
        with self._lock:
            self.power_enabled = status.power_enabled
            self.servo_enabled = status.servo_enabled
            self._power_feedback_time = status.power_updated_at
            self._servo_feedback_time = status.servo_updated_at

    def _gripper_ready_callback(self, msg: Bool) -> None:
        """Receive latched readiness from the gripper hardware driver."""

        previous = self._gripper_controller.status().ready
        value = bool(msg.data)
        failure = self._gripper_controller.update_ready(value)
        if value != previous:
            self._push_event(
                'info' if value else 'warning',
                f'Gripper driver readiness changed to {value}.',
            )
        if failure is not None:
            self._push_event('error', failure)
        if not value and self._active_gripper_task_id is not None:
            self._finish_active_gripper_task(
                TaskCommandStatus.FAILED,
                failure or 'gripper driver became not ready',
            )

    def _tool_flange_state_callback(self, side: str, msg) -> None:
        """Track measured tool-flange output voltage in volts."""

        if side not in self._tool_flange_voltages:
            return
        raw_voltage = float(msg.output_voltage)
        # The ROS message comment says millivolts, but the current rby1_driver
        # forwards the SDK field unchanged and the SDK reports whole volts
        # (0, 12, or 24). Accept both representations so this stays correct if
        # the ROS driver later aligns its implementation with the message.
        voltage = (
            raw_voltage / 1000.0
            if abs(raw_voltage) > 100.0
            else raw_voltage
        )
        if not math.isfinite(voltage):
            return
        with self._lock:
            self._tool_flange_voltages[side] = voltage
            self._tool_flange_updated_at[side] = time.monotonic()

    def _gripper_power_status(
        self,
        now: Optional[float] = None,
    ) -> Tuple[Optional[Tuple[float, float]], bool, bool]:
        """Return ([right, left] volts, feedback fresh, both at 12 V)."""

        current_time = time.monotonic() if now is None else float(now)
        with self._lock:
            right = self._tool_flange_voltages['right']
            left = self._tool_flange_voltages['left']
            right_updated = self._tool_flange_updated_at['right']
            left_updated = self._tool_flange_updated_at['left']

        voltages = (
            (right, left)
            if right is not None and left is not None
            else None
        )
        fresh = bool(
            voltages is not None
            and right_updated is not None
            and left_updated is not None
            and current_time - right_updated
            <= self.tool_flange_state_timeout_sec
            and current_time - left_updated
            <= self.tool_flange_state_timeout_sec
        )
        confirmed_12v = bool(
            fresh
            and voltages is not None
            and all(
                abs(value - GRIPPER_POWER_VOLTAGE)
                <= GRIPPER_POWER_TOLERANCE
                for value in voltages
            )
        )
        return voltages, fresh, confirmed_12v

    def _gripper_state_callback(self, msg: Float64MultiArray) -> None:
        """Receive measured right/left close ratios from the driver."""

        previous_error = self._gripper_controller.status().error
        try:
            completed = self._gripper_controller.update_state(msg.data)
        except (TypeError, ValueError) as exc:
            now = time.monotonic()
            if now - self._last_gripper_state_warning_at >= 2.0:
                self._push_event(
                    'warning',
                    f'Invalid gripper state ignored: {exc}',
                )
                self._last_gripper_state_warning_at = now
            return

        status = self._gripper_controller.status()
        if completed:
            self._push_event(
                'info',
                f'Gripper target reached: {list(status.positions or ())}.',
            )
            if (
                self._active_gripper_task_kind
                is CommandKind.GRIPPER_OPEN
            ):
                self._finish_active_gripper_task(
                    TaskCommandStatus.SUCCEEDED,
                    'Gripper open target reached',
                )
        elif status.error is not None and status.error != previous_error:
            self._push_event('error', status.error)
            if self._active_gripper_task_id is not None:
                self._finish_active_gripper_task(
                    TaskCommandStatus.FAILED,
                    status.error,
                )

    def _check_gripper_command(self) -> None:
        failure = self._gripper_controller.tick()
        if failure is not None:
            self._push_event('error', failure)
            if self._active_gripper_task_id is not None:
                self._finish_active_gripper_task(
                    TaskCommandStatus.FAILED,
                    failure,
                )
            return

        if (
            self._active_gripper_task_id is None
            or self._active_gripper_task_kind
            not in (CommandKind.GRIPPER_CLOSE, CommandKind.GRIPPER_SET)
            or self._active_gripper_settle_deadline is None
            or time.monotonic() < self._active_gripper_settle_deadline
        ):
            return

        marker = self._active_gripper_feedback_marker
        if marker is None:
            self._finish_active_gripper_task(
                TaskCommandStatus.FAILED,
                'gripper feedback marker is unavailable',
            )
            return
        try:
            self._gripper_controller.complete_after_settle(marker)
        except (GripperCommandError, TypeError, ValueError) as exc:
            self._finish_active_gripper_task(
                TaskCommandStatus.FAILED,
                str(exc),
            )
            return

        kind = self._active_gripper_task_kind
        label = 'close' if kind is CommandKind.GRIPPER_CLOSE else 'set'
        self._finish_active_gripper_task(
            TaskCommandStatus.SUCCEEDED,
            f'Gripper {label} settle time elapsed with fresh feedback',
        )

    def _state_callback(self, msg) -> None:
        """Receive the actual state published by the RB-Y1 driver."""

        self.robot_session.update_driver_state(msg)
        self._robot_state_received = True
        self._robot_state_updated_at = time.monotonic()

        new_control_state = int(msg.control_manager_state)
        new_stream_enabled = bool(msg.robot_stream_state)
        new_emo_active = bool(msg.emo_state)
        new_collision_active = bool(msg.collision)
        safety_stop_reason: Optional[str] = None

        if new_control_state != self.control_state:
            self.control_state = new_control_state

            self._push_event(
                'info',
                f'Robot control state changed to '
                f'{new_control_state}.',
            )
            if new_control_state in CONTROL_MANAGER_FAULT_STATES:
                safety_stop_reason = (
                    f'Control Manager entered fault state '
                    f'{new_control_state}'
                )

        if new_stream_enabled != self.stream_enabled:
            self.stream_enabled = new_stream_enabled

            self._push_event(
                'info',
                'Robot stream state changed to '
                f'{"ON" if new_stream_enabled else "OFF"}.',
            )

        if new_emo_active != self.emo_active:
            self.emo_active = new_emo_active

            self._push_event(
                'warning' if new_emo_active else 'info',
                'EMO state changed to '
                f'{"ACTIVE" if new_emo_active else "RELEASED"}.',
            )

            if new_emo_active:
                safety_stop_reason = 'EMO became active'

        if new_collision_active != self.collision_active:
            self.collision_active = new_collision_active

            self._push_event(
                'warning' if new_collision_active else 'info',
                'Collision state changed to '
                f'{"ACTIVE" if new_collision_active else "CLEAR"}.',
            )

            if new_collision_active:
                safety_stop_reason = 'collision state became active'

        if safety_stop_reason is not None:
            self._runtime_safety_stop_latched = True
            self._push_event(
                'error',
                f'Safety stop: {safety_stop_reason}.',
            )
            self.stop(publish_immediately=True)
            self.cancel_motion()
        elif (
            new_control_state in (2, 3)
            and not new_emo_active
            and not new_collision_active
        ):
            self._runtime_safety_stop_latched = False

    def _joint_state_callback(
        self,
        group: str,
        msg: JointState,
    ) -> None:
        """Cache a joint group state in operator-friendly degrees."""
        self.manipulator_controller.update_joint_state(group, msg)

    def _poll_cartesian_state(self) -> None:
        """Request current right/left end-effector poses asynchronously."""

        if (
            not self.services_enabled
            or self.cartesian_pose_client is None
            or GetCartesianPose is None
        ):
            return

        if not self.cartesian_pose_client.service_is_ready():
            return

        for arm in ('right_arm', 'left_arm'):
            if self._cartesian_request_pending[arm]:
                continue

            ref_link, target_link = self.cartesian_links[arm]

            request = GetCartesianPose.Request()
            request.ref_link = ref_link
            request.target_link = target_link

            future = self.cartesian_pose_client.call_async(
                request
            )

            self._cartesian_request_pending[arm] = True
            self._pending_futures.append(future)

            future.add_done_callback(
                lambda done, arm_name=arm:
                self._cartesian_pose_done(
                    done,
                    arm_name,
                )
            )

    def _transform_to_pose(self, transform) -> List[float]:
        quaternion = self._normalize_quaternion(
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
        )

        roll_deg, pitch_deg, yaw_deg = (
            self._quaternion_to_rpy_deg(
                *quaternion,
            )
        )

        return [
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
            roll_deg,
            pitch_deg,
            yaw_deg,
        ]

    def _cartesian_pose_done(
        self,
        future,
        arm: str,
    ) -> None:
        self._discard_future(future)
        self._cartesian_request_pending[arm] = False

        try:
            response = future.result()
        except Exception as exc:
            self._push_event(
                'warning',
                f'Cartesian pose request failed '
                f'for {arm}: {exc}',
            )
            return

        if response is None:
            return

        try:
            self.manipulator_controller.update_cartesian(
                arm,
                response.transform,
            )
        except ValueError as exc:
            self._push_event(
                'warning',
                f'Invalid Cartesian pose for {arm}: {exc}',
            )
            return

    def _cartesian_snapshot_done(
        self,
        future,
        arm: str,
        ) -> None:
        self._discard_future(future)
        self._cartesian_snapshot_pending[arm] = False

        try:
            response = future.result()
        except Exception as exc:
            self._push_event(
                'warning',
                f'Get TCP failed for {arm}: {exc}',
            )
            return

        if response is None:
            return

        try:
            pose = self._transform_to_pose(response.transform)
        except ValueError as exc:
            self._push_event(
                'warning',
                f'Invalid TCP snapshot for {arm}: {exc}',
            )
            return

        with self._lock:
            self._cartesian_snapshot[arm] = pose

    @staticmethod
    def _quaternion_to_rpy_deg(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> Tuple[float, float, float]:
        return ManipulatorController.quaternion_to_rpy_deg(x, y, z, w)

    @staticmethod
    def _rpy_deg_to_quaternion(
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> Tuple[float, float, float, float]:
        return ManipulatorController.rpy_deg_to_quaternion(
            roll_deg,
            pitch_deg,
            yaw_deg,
        )

    @staticmethod
    def _normalize_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> Tuple[float, float, float, float]:
        return ManipulatorController.normalize_quaternion(x, y, z, w)

    @staticmethod
    def _multiply_quaternions(
        left: Tuple[float, float, float, float],
        right: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        return ManipulatorController.multiply_quaternions(left, right)

    @staticmethod
    def _axis_angle_deg_to_quaternion(
        axis_index: int,
        angle_deg: float,
    ) -> Tuple[float, float, float, float]:
        return ManipulatorController.axis_angle_deg_to_quaternion(
            axis_index,
            angle_deg,
        )

    def get_motion_state(self) -> Dict[str, object]:
        """Return cached joint and Cartesian state for control clients."""
        return self.manipulator_controller.motion_state()

    def request_cartesian_snapshot(self, arm: str) -> bool:
        """Request a fresh Cartesian pose snapshot."""

        if arm not in self.cartesian_links:
            return False

        if (
            not self.services_enabled
            or self.cartesian_pose_client is None
            or GetCartesianPose is None
        ):
            return False

        if not self.cartesian_pose_client.service_is_ready():
            self._push_event(
                'warning',
                'GetCartesianPose service is not ready.',
            )
            return False

        if self._cartesian_snapshot_pending[arm]:
            return False

        ref_link, target_link = self.cartesian_links[arm]

        request = GetCartesianPose.Request()
        request.ref_link = ref_link
        request.target_link = target_link

        # 이전 snapshot을 지운다.
        with self._lock:
            self._cartesian_snapshot[arm] = None

        future = self.cartesian_pose_client.call_async(request)

        self._cartesian_snapshot_pending[arm] = True
        self._pending_futures.append(future)

        future.add_done_callback(
            lambda done, arm_name=arm:
            self._cartesian_snapshot_done(
                done,
                arm_name,
            )
        )

        return True

    def get_cartesian_snapshot(
        self,
        arm: str,
    ) -> Optional[List[float]]:

        with self._lock:
            values = self._cartesian_snapshot.get(arm)

            if values is None:
                return None

            return list(values)

    def get_joint_snapshot(
        self,
        group: str,
        ) -> Optional[List[float]]:
        """Return a copy of the latest joint state in degrees."""

        with self._lock:
            values = self._joint_groups_deg.get(group)

            if values is None:
                return None

        return list(values)

    @staticmethod
    def _timestamp_is_fresh(
        updated_at: Optional[float],
        *,
        now: Optional[float] = None,
        maximum_age: float = ROBOT_STATE_MAX_AGE_SEC,
    ) -> bool:
        if updated_at is None:
            return False
        captured_at = time.monotonic() if now is None else float(now)
        age = captured_at - float(updated_at)
        return math.isfinite(age) and 0.0 <= age <= maximum_age

    def _live_motion_safety_reason(
        self,
        *,
        now: Optional[float] = None,
    ) -> Optional[str]:
        """Return why live robot state is unsafe, or ``None`` if clear."""

        return self.state_manager.safety_reason(
            maximum_age_sec=ROBOT_STATE_MAX_AGE_SEC,
            now=now,
        )

    def _base_motion_safety_reason(self) -> Optional[str]:
        if not self.services_enabled:
            return None
        return self._live_motion_safety_reason()

    def _joint_feedback_ready(self, group: str) -> bool:
        now = time.monotonic()
        with self._lock:
            values = self._joint_groups_deg.get(group)
            updated_at = self._joint_updated_at.get(group)
            order_verified = self._joint_order_verified.get(group, False)
        if (
            values is None
            or not order_verified
            or not self._timestamp_is_fresh(
                updated_at,
                now=now,
                maximum_age=MANIPULATOR_STATE_MAX_AGE_SEC,
            )
        ):
            self._push_event(
                'error',
                f'Motion rejected: {group} Joint state is unavailable, '
                'stale, or not name-ordered.',
            )
            return False
        return True

    def _cartesian_feedback_ready(self, arm: str) -> bool:
        now = time.monotonic()
        with self._lock:
            values = self._cartesian_state.get(arm)
            updated_at = self._cartesian_updated_at.get(arm)
        if (
            values is None
            or not self._timestamp_is_fresh(
                updated_at,
                now=now,
                maximum_age=MANIPULATOR_STATE_MAX_AGE_SEC,
            )
        ):
            self._push_event(
                'error',
                f'Motion rejected: {arm} Cartesian state is unavailable '
                'or stale.',
            )
            return False
        return True

    def _enforce_runtime_motion_safety(self) -> None:
        """Stop UI-owned motion if live safety feedback is lost or unsafe."""

        if not self.services_enabled:
            return

        command, command_stale = self._current_command()
        with self._lock:
            motion_active = bool(self._motion_busy)
        base_active = not command.stopped and not command_stale

        if not motion_active and not base_active:
            return

        reason = self._live_motion_safety_reason()
        if reason is None:
            self._runtime_safety_stop_latched = False
            return
        if self._runtime_safety_stop_latched:
            return

        self._runtime_safety_stop_latched = True
        self._push_event('error', f'Runtime safety stop: {reason}.')
        self.stop(publish_immediately=True)
        self.cancel_motion()

    def _motion_command_allowed(self) -> bool:
        """Common safety gate before sending a manipulation goal."""

        if not self.services_enabled:
            self._push_event(
                'warning',
                'RB-Y1 manipulation interfaces are disabled.',
            )
            return False

        safety_reason = self._live_motion_safety_reason()
        if safety_reason is not None:
            self._push_event(
                'error',
                f'Motion rejected: {safety_reason}.',
            )
            return False

        if self._motion_busy:
            self._push_event(
                'warning',
                'Motion rejected: another Joint/Cartesian '
                'command is still active.',
            )
            return False

        return True

    def jog_joint(
        self,
        group: str,
        index: int,
        delta_deg: float,
    ) -> None:
        """Jog one joint by sending a full-group absolute target."""

        with self._lock:
            current = self._joint_groups_deg.get(group)
            current_copy = (
                list(current)
                if current is not None
                else None
            )

        if current_copy is None:
            self._push_event(
                'warning',
                f'Joint jog rejected: no state for {group}.',
            )
            return

        if index < 0 or index >= len(current_copy):
            self._push_event(
                'error',
                f'Joint jog rejected: invalid index {index}.',
            )
            return

        current_copy[index] += float(delta_deg)

        self.move_joint_group(
            group,
            current_copy,
            self.joint_jog_minimum_time_sec,
        )

    def _validate_joint_targets(
        self,
        group: str,
        targets_deg: List[float],
    ) -> bool:
        """Check requested joint targets against RB-Y1 M v1.3 limits."""

        limits = JOINT_LIMITS_RAD.get(group)

        if limits is None:
            self._push_event(
                "error",
                f"No joint limits defined for group: {group}.",
            )
            return False

        if len(targets_deg) != len(limits):
            self._push_event(
                "error",
                f"{group} target count does not match joint limits.",
            )
            return False

        try:
            numeric_targets = [float(value) for value in targets_deg]
        except (TypeError, ValueError):
            self._push_event(
                "error",
                f"{group} target contains a non-numeric value.",
            )
            return False
        if not all(math.isfinite(value) for value in numeric_targets):
            self._push_event(
                "error",
                f"{group} target contains a non-finite value.",
            )
            return False

        for index, (target_deg, limit) in enumerate(
            zip(numeric_targets, limits)
        ):
            lower_rad, upper_rad = limit
            target_rad = math.radians(float(target_deg))

            if (
                target_rad < lower_rad
                or target_rad > upper_rad
            ):
                lower_deg = math.degrees(lower_rad)
                upper_deg = math.degrees(upper_rad)

                joint_name = f"{group}_{index}"

                self._push_event(
                    "warning",
                    "Motion rejected: "
                    f"{joint_name} target "
                    f"{target_deg:+.2f} deg is outside "
                    f"[{lower_deg:+.2f}, "
                    f"{upper_deg:+.2f}] deg.",
                )

                return False

        return True

    def move_joint_group(
        self,
        group: str,
        targets_deg: List[float],
        minimum_time: float,
    ) -> None:
        """Send an absolute joint-position command.

        GUI values are degrees.  The ROS action receives radians.
        """

        expected_dof = {
            'right_arm': 7,
            'left_arm': 7,
            'torso': 6,
            'head': 2,
        }

        dof = expected_dof.get(group)
        if dof is None:
            self._push_event(
                'error',
                f'Unknown joint group: {group}.',
            )
            return

        if len(targets_deg) != dof:
            self._push_event(
                'error',
                f'{group} expects {dof} targets, '
                f'got {len(targets_deg)}.',
            )
            return

        try:
            values_deg = [float(value) for value in targets_deg]
            minimum_time_value = float(minimum_time)
        except (TypeError, ValueError):
            self._push_event(
                'error',
                'Non-numeric Joint command value rejected.',
            )
            return

        if (
            not all(math.isfinite(v) for v in values_deg)
            or not math.isfinite(minimum_time_value)
            or minimum_time_value <= 0.0
        ):
            self._push_event(
                'error',
                'Non-finite Joint target or invalid minimum time rejected.',
            )
            return

        if not self._validate_joint_targets(
            group,
            values_deg,
        ):
            return

        if not self._motion_command_allowed():
            return
        if not self._joint_feedback_ready(group):
            return

        if (
            self.joint_action_client is None
            or Rby1JointCommand is None
            or JointCommand is None
        ):
            self._push_event(
                'error',
                'Joint action client is unavailable.',
            )
            return

        if not self.joint_action_client.server_is_ready():
            self._push_event(
                'warning',
                f'Joint action server not ready: '
                f'{self.joint_action_name}',
            )
            return

        command = JointCommand()
        command.position = [
            math.radians(value)
            for value in values_deg
        ]
        command.minimum_time = max(
            0.1,
            minimum_time_value,
        )
        command.velocity_limit = self.manual_joint_velocity_limit
        command.acceleration_limit = self.manual_joint_acceleration_limit
        command.use_impedance = False
        command.use_group_joint = False

        goal = Rby1JointCommand.Goal()
        setattr(goal, group, command)

        self._send_motion_goal(
            client=self.joint_action_client,
            goal=goal,
            label=f'Joint {group}',
        )

    def jog_cartesian(
        self,
        arm: str,
        axis_index: int,
        delta: float,
        reference_frame: str = 'base',
    ) -> None:
        """Jog one Cartesian component from the latest measured pose.

        Linear jogs add to translation in the configured reference frame.
        Angular jogs compose a delta quaternion on the left, so Roll/Pitch/Yaw
        consistently mean rotations about the reference-frame X/Y/Z axes.
        """

        with self._lock:
            current = self._cartesian_state.get(arm)
            current_quaternion = (
                self._cartesian_quaternion_state.get(arm)
                if hasattr(self, '_cartesian_quaternion_state')
                else None
            )

            snapshot = (
                self._cartesian_snapshot.get(arm)
                if hasattr(self, "_cartesian_snapshot")
                else None
            )

            target = (
                list(current)
                if current is not None
                else None
            )

        self._push_event(
            "info",
            f"[CS SOURCE] {arm} "
            f"live={current}, "
            f"snapshot={snapshot}",
        )

        if target is None:
            self._push_event(
                'warning',
                f'Cartesian jog rejected: '
                f'no current pose for {arm}.',
            )
            return

        if axis_index < 0 or axis_index >= 6:
            self._push_event(
                'error',
                f'Cartesian jog rejected: '
                f'invalid axis {axis_index}.',
            )
            return

        #---Log A: log the current and target pose---
        current_pose = list(target)

        if current_quaternion is None:
            self._push_event(
                'warning',
                f'Cartesian jog rejected: '
                f'no current quaternion for {arm}.',
            )
            return

        translation = list(current_pose[:3])

        if axis_index < 3:
            translation[axis_index] += float(delta)
            target_quaternion = current_quaternion
        else:
            delta_quaternion = self._axis_angle_deg_to_quaternion(
                axis_index - 3,
                float(delta),
            )

            # Left multiplication applies the delta around an axis of the
            # reference frame (Base in the current UI).
            composed = self._multiply_quaternions(
                delta_quaternion,
                current_quaternion,
            )
            target_quaternion = self._normalize_quaternion(*composed)

        target_rpy = self._quaternion_to_rpy_deg(*target_quaternion)
        target = translation + list(target_rpy)

        axis_names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]

        self._push_event(
            "info",
            f"[CS JOG] {arm} "
            f"axis={axis_names[axis_index]}, "
            f"current={['%.3f' % v for v in current_pose]}, "
            f"target={['%.3f' % v for v in target]}",
        )

        self._move_cartesian_quaternion(
            arm,
            translation,
            target_quaternion,
            self.cartesian_jog_minimum_time_sec,
            reference_frame='base',
            log_rpy=target_rpy,
        )

    def move_cartesian(
        self,
        arm: str,
        target: List[float],
        minimum_time: float,
        reference_frame: str = 'base',
    ) -> None:
        """Send an absolute Cartesian target.

        target = [x, y, z, roll_deg, pitch_deg, yaw_deg]
        """

        if arm not in ('right_arm', 'left_arm'):
            self._push_event(
                'error',
                f'Unknown Cartesian arm: {arm}.',
            )
            return

        if len(target) != 6:
            self._push_event(
                'error',
                'Cartesian target must contain 6 values.',
            )
            return

        try:
            values = [float(value) for value in target]
        except (TypeError, ValueError):
            self._push_event(
                'error',
                'Non-numeric Cartesian target rejected.',
            )
            return

        if not all(math.isfinite(v) for v in values):
            self._push_event(
                'error',
                'Non-finite Cartesian target rejected.',
            )
            return

        quaternion = self._rpy_deg_to_quaternion(
            values[3],
            values[4],
            values[5],
        )

        self._move_cartesian_quaternion(
            arm,
            values[:3],
            quaternion,
            minimum_time,
            reference_frame=reference_frame,
            log_rpy=(values[3], values[4], values[5]),
        )

    def _move_cartesian_quaternion(
        self,
        arm: str,
        translation: List[float],
        quaternion: Tuple[float, float, float, float],
        minimum_time: float,
        reference_frame: str = 'base',
        log_rpy: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        """Send an absolute Cartesian target using a quaternion directly."""

        if arm not in ('right_arm', 'left_arm'):
            self._push_event(
                'error',
                f'Unknown Cartesian arm: {arm}.',
            )
            return

        if len(translation) != 3 or len(quaternion) != 4:
            self._push_event(
                'error',
                'Cartesian quaternion target must contain 3 translation '
                'and 4 rotation values.',
            )
            return

        try:
            position = [float(value) for value in translation]
            minimum_time_value = float(minimum_time)
        except (TypeError, ValueError):
            self._push_event(
                'error',
                'Non-numeric Cartesian command value rejected.',
            )
            return

        if (
            not all(math.isfinite(value) for value in position)
            or not math.isfinite(minimum_time_value)
            or minimum_time_value <= 0.0
        ):
            self._push_event(
                'error',
                'Non-finite Cartesian target or invalid minimum time '
                'rejected.',
            )
            return

        try:
            qx, qy, qz, qw = self._normalize_quaternion(*quaternion)
        except ValueError as exc:
            self._push_event(
                'error',
                f'Invalid Cartesian quaternion rejected: {exc}.',
            )
            return

        if not self._motion_command_allowed():
            return
        if not self._cartesian_feedback_ready(arm):
            return

        if (
            self.cartesian_action_client is None
            or Rby1CartesianCommand is None
            or CartesianCommand is None
        ):
            self._push_event(
                'error',
                'Cartesian action client is unavailable.',
            )
            return

        if not self.cartesian_action_client.server_is_ready():
            self._push_event(
                'warning',
                f'Cartesian action server not ready: '
                f'{self.cartesian_action_name}',
            )
            return

        configured_ref, target_link = self.cartesian_links[arm]

        # The current UI exposes Base as the operator reference frame.
        # Keep the verified configured link unless a future UI explicitly
        # introduces a selectable frame.
        ref_link = (
            configured_ref
            if reference_frame == 'base'
            else str(reference_frame)
        )

        command = CartesianCommand()
        command.ref_link = ref_link
        command.target_link = target_link

        command.transform.translation.x = position[0]
        command.transform.translation.y = position[1]
        command.transform.translation.z = position[2]

        if log_rpy is None:
            log_rpy = self._quaternion_to_rpy_deg(qx, qy, qz, qw)

        #---Log B: log the target pose in RPY and quaternion---#
        self._push_event(
            "info",
            f"[CS QUAT] {arm} "
            f"RPY=({log_rpy[0]:+.3f}, "
            f"{log_rpy[1]:+.3f}, "
            f"{log_rpy[2]:+.3f}) deg, "
            f"Q=({qx:+.6f}, {qy:+.6f}, "
            f"{qz:+.6f}, {qw:+.6f})",
        )
        command.transform.rotation.x = qx
        command.transform.rotation.y = qy
        command.transform.rotation.z = qz
        command.transform.rotation.w = qw

        command.minimum_time = max(
            0.1,
            minimum_time_value,
        )
        command.linear_velocity_limit = (
            self.manual_cartesian_linear_velocity_limit
        )
        command.angular_velocity_limit = (
            self.manual_cartesian_angular_velocity_limit
        )
        command.acceleration_limit_scaling = (
            self.manual_cartesian_acceleration_scaling
        )
        command.use_impedance = False

        goal = Rby1CartesianCommand.Goal()
        setattr(goal, arm, command)

        self._send_motion_goal(
            client=self.cartesian_action_client,
            goal=goal,
            label=f'Cartesian {arm}',
        )

    # ==================================================================
    # Asynchronous Scenario Task contract
    # ==================================================================
    def task_state(self) -> TaskBackendState:
        now = time.monotonic()
        robot_status = self.robot_session.snapshot()
        with self._lock:
            return TaskBackendState(
                captured_at=now,
                robot_state_updated_at=robot_status.robot_state_updated_at,
                control_state=robot_status.control_state,
                stream_enabled=robot_status.stream_enabled,
                emo_active=robot_status.emo_active,
                collision_active=robot_status.collision_active,
                motion_active=bool(
                    self._motion_busy
                    or self._active_gripper_task_id is not None
                ),
                joint_groups={
                    group: tuple(values) if values is not None else None
                    for group, values in self._joint_groups_deg.items()
                },
                joint_updated_at=dict(self._joint_updated_at),
                joint_order_verified=dict(self._joint_order_verified),
                cartesian={
                    arm: tuple(values) if values is not None else None
                    for arm, values in self._cartesian_state.items()
                },
                cartesian_updated_at=dict(self._cartesian_updated_at),
                # Compatibility fields retained in the shared contract. The
                # restored stock driver exposes no Scenario capability service.
                driver_safety_verified=False,
                driver_safety_updated_at=None,
            )

    def start_task_command(self, command: TaskCommand) -> str:
        """Validate and send one already-resolved absolute Task command."""

        if not isinstance(command, TaskCommand):
            raise TypeError('command must be a TaskCommand')
        if command.kind in (
            CommandKind.GRIPPER_OPEN,
            CommandKind.GRIPPER_CLOSE,
            CommandKind.GRIPPER_SET,
        ):
            return self._start_gripper_task_command(command)
        if command.kind not in (
            CommandKind.JOINT_ABSOLUTE,
            CommandKind.JOINT_ABSOLUTE_MULTI,
            CommandKind.LINEAR_ABSOLUTE,
        ):
            raise ValueError('backend accepts only resolved absolute commands')
        if not self._motion_command_allowed():
            raise RuntimeError('robot rejected the Task motion command')

        self._task_command_sequence += 1
        command_id = f'task-{self._task_command_sequence}'
        self._task_commands[command_id] = TaskCommandState(
            TaskCommandStatus.PENDING,
            'Goal request pending',
        )

        if command.kind in (
            CommandKind.JOINT_ABSOLUTE,
            CommandKind.JOINT_ABSOLUTE_MULTI,
        ):
            if (
                self.joint_action_client is None
                or Rby1JointCommand is None
                or JointCommand is None
                or not self.joint_action_client.server_is_ready()
            ):
                raise RuntimeError('Joint action server is unavailable')

            joint_targets = (
                command.joint_targets
                if command.kind is CommandKind.JOINT_ABSOLUTE_MULTI
                else ((str(command.group), command.values),)
            )

            goal = Rby1JointCommand.Goal()
            group_labels = []
            for group, target_values in joint_targets:
                if not self._joint_feedback_ready(group):
                    raise RuntimeError(
                        f'fresh, name-ordered {group} Joint state is required'
                    )
                values_deg = list(target_values)
                if not self._validate_joint_targets(group, values_deg):
                    self._task_commands[command_id] = TaskCommandState(
                        TaskCommandStatus.FAILED,
                        f'{group} target violates a hard limit',
                    )
                    raise ValueError(
                        f'Task {group} target violates a hard limit'
                    )

                velocity_limits = JOINT_MAX_VELOCITY_RAD.get(group)
                acceleration_limits = JOINT_MAX_ACCELERATION_RAD.get(group)
                if (
                    velocity_limits is None
                    or float(command.velocity_limit) > min(velocity_limits)
                ):
                    raise ValueError(
                        f'Task {group} velocity_limit exceeds the '
                        'verified model limit'
                    )
                if (
                    acceleration_limits is None
                    or any(value is None for value in acceleration_limits)
                    or float(command.acceleration_limit) > min(
                        float(value)
                        for value in acceleration_limits
                        if value is not None
                    )
                ):
                    raise ValueError(
                        f'Task {group} acceleration_limit is '
                        'unverified or too high'
                    )

                message = JointCommand()
                message.position = [
                    math.radians(value) for value in values_deg
                ]
                message.minimum_time = float(command.minimum_time)
                message.velocity_limit = float(command.velocity_limit)
                message.acceleration_limit = float(
                    command.acceleration_limit
                )
                message.use_impedance = False
                message.use_group_joint = False
                setattr(goal, group, message)
                group_labels.append(group)

            label = ' + '.join(group_labels)
            self._send_motion_goal(
                client=self.joint_action_client,
                goal=goal,
                label=f'Task Joint {label}',
                task_id=command_id,
            )
        else:
            arm = str(command.group)
            if arm not in self.cartesian_links:
                raise ValueError(f'unknown Cartesian arm: {arm}')
            if not self._cartesian_feedback_ready(arm):
                raise RuntimeError(
                    f'fresh {arm} Cartesian state is required'
                )
            if (
                self.cartesian_action_client is None
                or Rby1CartesianCommand is None
                or CartesianCommand is None
                or not self.cartesian_action_client.server_is_ready()
            ):
                raise RuntimeError('Cartesian action server is unavailable')

            ref_link, target_link = self.cartesian_links[arm]
            values = command.values
            message = CartesianCommand()
            message.ref_link = ref_link
            message.target_link = target_link
            message.transform.translation.x = values[0]
            message.transform.translation.y = values[1]
            message.transform.translation.z = values[2]
            qx, qy, qz, qw = self._rpy_deg_to_quaternion(*values[3:6])
            message.transform.rotation.x = qx
            message.transform.rotation.y = qy
            message.transform.rotation.z = qz
            message.transform.rotation.w = qw
            message.minimum_time = float(command.minimum_time)
            message.linear_velocity_limit = float(command.linear_velocity)
            message.angular_velocity_limit = float(command.angular_velocity)
            message.acceleration_limit_scaling = float(
                command.acceleration_scaling
            )
            # Scenario Cartesian commands use the SDK builder path.
            message.use_impedance = False

            goal = Rby1CartesianCommand.Goal()
            goal.stop_position_tracking_error = (
                TASK_CARTESIAN_STOP_POSITION_ERROR_M
            )
            goal.stop_orientation_tracking_error = (
                TASK_CARTESIAN_STOP_ORIENTATION_ERROR_RAD
            )
            setattr(goal, arm, message)
            self._send_motion_goal(
                client=self.cartesian_action_client,
                goal=goal,
                label=f'Task Cartesian {arm}',
                task_id=command_id,
            )

        return command_id

    def _start_gripper_task_command(self, command: TaskCommand) -> str:
        if self._active_gripper_task_id is not None:
            raise RuntimeError('another gripper Task command is active')

        self._task_command_sequence += 1
        command_id = f'task-{self._task_command_sequence}'
        self._task_commands[command_id] = TaskCommandState(
            TaskCommandStatus.PENDING,
            'Gripper command pending',
        )

        status_before = self._gripper_controller.status()
        try:
            if command.kind is CommandKind.GRIPPER_OPEN:
                target = self._gripper_controller.command(
                    'open',
                    str(command.group),
                )
            elif command.kind is CommandKind.GRIPPER_CLOSE:
                target = self._gripper_controller.command(
                    'close',
                    str(command.group),
                )
            elif command.kind is CommandKind.GRIPPER_SET:
                target = self._gripper_controller.command_ratio(
                    float(command.values[0]),
                    str(command.group),
                )
            else:  # Guarded by start_task_command(), retained defensively.
                raise ValueError('unsupported gripper Task command')
            self._publish_gripper_target(target)
        except Exception as exc:
            self._task_commands[command_id] = TaskCommandState(
                TaskCommandStatus.FAILED,
                str(exc),
            )
            raise

        status_after = self._gripper_controller.status()
        label = command.kind.value
        self._push_event(
            'info',
            f'Task {label} requested for {command.group}: '
            f'target={list(target)}.',
        )

        if (
            command.kind is CommandKind.GRIPPER_OPEN
            and not status_after.motion_active
        ):
            self._task_commands[command_id] = TaskCommandState(
                TaskCommandStatus.SUCCEEDED,
                'Gripper was already at the open target',
            )
            return command_id

        self._active_gripper_task_id = command_id
        self._active_gripper_task_kind = command.kind
        self._active_gripper_feedback_marker = (
            status_before.feedback_sequence
        )
        self._active_gripper_settle_deadline = (
            time.monotonic() + float(command.seconds)
            if command.kind in (
                CommandKind.GRIPPER_CLOSE,
                CommandKind.GRIPPER_SET,
            )
            else None
        )
        return command_id

    def poll_task_command(self, command_id: str) -> TaskCommandState:
        try:
            return self._task_commands[command_id]
        except KeyError as exc:
            raise KeyError(f'unknown Task command: {command_id}') from exc

    def cancel_task_command(self, command_id: str) -> None:
        if command_id == self._active_gripper_task_id:
            hold_target = self._gripper_controller.cancel_and_hold()
            if hold_target is not None:
                self._publish_gripper_target(hold_target)
            self._set_task_command_result(
                command_id,
                TaskCommandStatus.CANCELED,
                (
                    'Canceled by operator; holding current gripper position'
                    if hold_target is not None
                    else 'Canceled by operator; fresh hold position unavailable'
                ),
            )
            self._clear_active_gripper_task()
            return

        current = self._task_commands.get(command_id)
        if current is not None and current.status is TaskCommandStatus.PENDING:
            self._task_commands[command_id] = TaskCommandState(
                TaskCommandStatus.CANCELED,
                'Canceled by operator',
            )
        self.cancel_motion()

    def _finish_active_gripper_task(
        self,
        status: TaskCommandStatus,
        message: str,
    ) -> None:
        command_id = self._active_gripper_task_id
        if status is TaskCommandStatus.FAILED:
            hold_target = self._gripper_controller.cancel_and_hold()
            if hold_target is not None:
                self._publish_gripper_target(hold_target)
        self._set_task_command_result(command_id, status, message)
        if command_id is not None:
            level = (
                'info'
                if status is TaskCommandStatus.SUCCEEDED
                else 'error'
            )
            self._push_event(level, message)
        self._clear_active_gripper_task()

    def _clear_active_gripper_task(self) -> None:
        self._active_gripper_task_id = None
        self._active_gripper_task_kind = None
        self._active_gripper_feedback_marker = None
        self._active_gripper_settle_deadline = None

    def _set_task_command_result(
        self,
        command_id: Optional[str],
        status: TaskCommandStatus,
        message: str,
    ) -> None:
        if command_id is None:
            return
        current = self._task_commands.get(command_id)
        if current is None or current.status is not TaskCommandStatus.PENDING:
            return
        self._task_commands[command_id] = TaskCommandState(status, message)

    def _send_motion_goal(
        self,
        client,
        goal,
        label: str,
        task_id: Optional[str] = None,
    ) -> None:
        self._motion_busy = True
        self._active_motion_kind = label
        self._active_goal_handle = None
        self._active_task_command_id = task_id
        self._cancel_motion_on_accept = False
        self._active_cancel_requested = False

        future = client.send_goal_async(goal)
        self._pending_futures.append(future)

        future.add_done_callback(
            lambda done, motion_label=label, command_id=task_id:
            self._motion_goal_response(
                done,
                motion_label,
                command_id,
            )
        )

        self._push_event(
            'info',
            f'{label} goal requested.',
        )

    def _motion_goal_response(
        self,
        future,
        label: str,
        task_id: Optional[str] = None,
    ) -> None:
        self._discard_future(future)

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._motion_busy = False
            self._active_motion_kind = None
            self._active_goal_handle = None
            self._active_task_command_id = None
            self._cancel_motion_on_accept = False
            self._active_cancel_requested = False
            self._set_task_command_result(
                task_id,
                TaskCommandStatus.FAILED,
                str(exc),
            )

            self._push_event(
                'error',
                f'{label} goal request failed: {exc}',
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            self._motion_busy = False
            self._active_motion_kind = None
            self._active_goal_handle = None
            self._active_task_command_id = None
            self._cancel_motion_on_accept = False
            self._active_cancel_requested = False
            self._set_task_command_result(
                task_id,
                TaskCommandStatus.FAILED,
                'Goal rejected',
            )

            self._push_event(
                'error',
                f'{label} goal was rejected.',
            )
            return

        self._active_goal_handle = goal_handle

        self._push_event(
            'info',
            f'{label} goal accepted.',
        )

        # The direction key may have been released while send_goal_async()
        # was still waiting for the server response.
        if self._cancel_motion_on_accept:
            self._request_active_goal_cancel(goal_handle)

        result_future = goal_handle.get_result_async()
        self._pending_futures.append(result_future)

        result_future.add_done_callback(
            lambda done, motion_label=label, command_id=task_id:
            self._motion_result_done(
                done,
                motion_label,
                command_id,
            )
        )

    def _motion_result_done(
        self,
        future,
        label: str,
        task_id: Optional[str] = None,
    ) -> None:
        self._discard_future(future)

        self._motion_busy = False
        self._active_motion_kind = None
        self._active_goal_handle = None
        self._active_task_command_id = None
        self._cancel_motion_on_accept = False
        self._active_cancel_requested = False

        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as exc:
            self._set_task_command_result(
                task_id,
                TaskCommandStatus.FAILED,
                str(exc),
            )
            self._push_event(
                'error',
                f'{label} result failed: {exc}',
            )
            return

        if bool(result.success):
            self._set_task_command_result(
                task_id,
                TaskCommandStatus.SUCCEEDED,
                str(result.finish_code),
            )
            self._push_event(
                'info',
                f'{label} completed: '
                f'{result.finish_code}',
            )
        else:
            self._set_task_command_result(
                task_id,
                TaskCommandStatus.FAILED,
                str(result.finish_code),
            )
            self._push_event(
                'warning',
                f'{label} finished unsuccessfully: '
                f'{result.finish_code}',
            )

    def is_motion_busy(self) -> bool:
        """Return whether a Joint/Cartesian action is currently active."""
        return bool(self._motion_busy)

    def cancel_active_motion(self) -> None:
        """Cancel only the current action goal, preserving stream state."""

        if not self._motion_busy:
            self._cancel_motion_on_accept = False
            return

        self._cancel_motion_on_accept = True

        goal_handle = self._active_goal_handle
        if goal_handle is None:
            return

        self._request_active_goal_cancel(goal_handle)

    def _request_active_goal_cancel(self, goal_handle) -> None:
        if self._active_cancel_requested:
            return

        self._active_cancel_requested = True

        try:
            future = goal_handle.cancel_goal_async()
        except Exception as exc:
            self._active_cancel_requested = False
            self._push_event(
                'warning',
                f'Active motion cancel request failed: {exc}',
            )
            return

        self._pending_futures.append(future)
        future.add_done_callback(
            self._active_motion_cancel_done
        )

    def _active_motion_cancel_done(self, future) -> None:
        self._discard_future(future)

        try:
            response = future.result()
        except Exception as exc:
            self._push_event(
                'warning',
                f'Active motion cancel failed: {exc}',
            )
            return

        goals_canceling = getattr(
            response,
            'goals_canceling',
            [],
        )

        if goals_canceling:
            self._push_event(
                'info',
                'Active motion cancel accepted.',
            )
        else:
            self._push_event(
                'warning',
                'Active motion cancel was not accepted.',
            )

    def cancel_motion(self) -> None:
        """Cancel robot control through the driver's Trigger service.

        The driver's cancel_control service is intentionally used here as
        the safety-oriented common cancel path for Joint/Cartesian control.
        """

        if self.cancel_control_client is None:
            return

        if not self.cancel_control_client.service_is_ready():
            self._push_event(
                'warning',
                f'Cancel service not ready: '
                f'{self.cancel_control_service}',
            )
            return

        request = Trigger.Request()
        future = self.cancel_control_client.call_async(request)

        self._pending_futures.append(future)

        future.add_done_callback(
            self._cancel_motion_done
        )

        self._push_event(
            'warning',
            'Motion cancel requested.',
        )

    def _cancel_motion_done(self, future) -> None:
        self._discard_future(future)

        try:
            result = future.result()
        except Exception as exc:
            self._push_event(
                'error',
                f'Motion cancel failed: {exc}',
            )
            return

        if result is not None and bool(result.success):
            active_task_id = self._active_task_command_id
            self._motion_busy = False
            self._active_motion_kind = None
            self._active_goal_handle = None
            self._active_task_command_id = None
            self._cancel_motion_on_accept = False
            self._active_cancel_requested = False
            self._set_task_command_result(
                active_task_id,
                TaskCommandStatus.CANCELED,
                str(result.message),
            )

            self._push_event(
                'info',
                f'Motion cancel succeeded: '
                f'{result.message}',
            )
        else:
            message = (
                getattr(result, 'message', 'No response')
                if result is not None
                else 'No response'
            )
            self._push_event(
                'error',
                f'Motion cancel failed: {message}',
            )

    def command_gripper(
        self,
        action: str,
        side: str = 'both',
    ) -> Tuple[float, float]:
        """Send a validated gripper preset to the hardware driver."""

        if self._active_gripper_task_id is not None:
            raise RuntimeError(
                'manual gripper commands are disabled while a gripper Task '
                'command is active'
            )
        target = self._gripper_controller.command(action, side)
        self._publish_gripper_target(target)
        self._push_event(
            'info',
            f'Gripper {str(action).lower()} requested for '
            f'{str(side).lower()}: target={list(target)}.',
        )
        return target

    def _publish_gripper_target(
        self,
        target: Tuple[float, float],
    ) -> None:
        message = Float64MultiArray()
        message.data = [target[0], target[1]]
        self.gripper_command_pub.publish(message)

    def set_velocity(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        """Update the target mobile-base velocity."""

        try:
            self.mobile_base_controller.set_velocity(vx, vy, wz)
        except ValueError:
            self._push_event(
                'error',
                'Invalid non-finite velocity command rejected.',
            )

    def stop(
        self,
        publish_immediately: bool = True,
    ) -> None:
        """Set the velocity command to zero."""

        self.mobile_base_controller.stop(
            publish_immediately=publish_immediately,
        )

    def _current_command(
        self,
    ) -> Tuple[VelocityCommand, bool]:
        return self.mobile_base_controller.current_command()

    def _publish_cycle(self) -> None:
        """Publish Twist at the configured fixed rate."""

        self.mobile_base_controller.publish_cycle()

    def _publish_twist(
        self,
        command: VelocityCommand,
    ) -> None:
        """Convert VelocityCommand to geometry_msgs/Twist."""

        self.mobile_base_controller.publish(command)

    def prepare_robot(self) -> None:
        """Run Power ON -> Servo ON as a non-blocking sequence."""

        if not self.services_enabled:
            self._push_event(
                'warning',
                'RB-Y1 service control is disabled.',
            )
            return

        if self._prepare_stage != 'idle':
            self._push_event(
                'warning',
                'Robot preparation is already running.',
            )
            return

        if self.control_state in (2, 3):
            self._push_event(
                'info',
                'Robot is already powered and servo-enabled.',
            )
            return

        self._prepare_stage = 'power_request'
        self._prepare_deadline = (
            time.monotonic() + 20.0
        )

        self._push_event(
            'info',
            'Starting robot preparation: '
            'Power ON -> Servo ON.',
        )

    def request_power(
        self,
        enabled: bool,
        target: str = 'all',
    ) -> None:
        """Request robot power ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.power_client,
            enabled=enabled,
            parameters=target,
            value=0.0,
            label='Power',
        )

    def request_servo(
        self,
        enabled: bool,
        target: str = 'all',
    ) -> None:
        """Request robot servo ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.servo_client,
            enabled=enabled,
            parameters=target,
            value=0.0,
            label='Servo',
        )

    def request_stream(
        self,
        enabled: bool,
        value: float = 0.0,
    ) -> None:
        """Request cmd_vel stream control ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.stream_client,
            enabled=enabled,
            parameters='',
            value=value,
            label='Stream',
        )

    def request_control_manager(
        self,
        command: str,
    ) -> None:
        """Send ENABLE, DISABLE, or RESET to the Control Manager."""

        if (
            not self.services_enabled
            or self.control_manager_client is None
            or ControlManagerCommand is None
        ):
            self._push_event(
                'warning',
                'Control Manager service is unavailable.',
            )
            return

        if not self.control_manager_client.service_is_ready():
            self._push_event(
                'warning',
                'Control Manager service not ready: '
                f'{self.control_manager_service}',
            )
            return

        command_key = str(command).strip().lower()
        command_values = {
            'enable': ControlManagerCommand.Request.CMD_ENABLE,
            'disable': ControlManagerCommand.Request.CMD_DISABLE,
            'reset': ControlManagerCommand.Request.CMD_RESET,
        }

        command_value = command_values.get(command_key)
        if command_value is None:
            self._push_event(
                'error',
                f'Unknown Control Manager command: {command}',
            )
            return

        # Stop locally commanded base velocity before commands that can
        # remove/reset control authority. The driver handles its own stream
        # shutdown for DISABLE / RESET.
        if command_key in ('disable', 'reset'):
            self.stop(publish_immediately=True)

        request = ControlManagerCommand.Request()
        request.command = int(command_value)

        future = self.control_manager_client.call_async(request)
        self._pending_futures.append(future)
        future.add_done_callback(
            lambda done, requested=command_key:
            self._control_manager_done(done, requested)
        )

        self._push_event(
            'info',
            f'Control Manager {command_key.upper()} requested.',
        )

    def request_gripper_power(self, enabled: bool) -> None:
        """Apply fixed 12 V to both tool flanges, or remove their power."""

        if enabled:
            self._request_state_on_off(
                client=self.tool_flange_power_client,
                enabled=True,
                parameters='12v',
                value=0.0,
                label='Gripper 12 V',
            )
            return

        if (
            not self.services_enabled
            or self.tool_flange_power_client is None
            or not self.tool_flange_power_client.service_is_ready()
        ):
            self._push_event(
                'warning',
                'Gripper 12 V service is unavailable; power was not changed.',
            )
            return

        with self._lock:
            if self._gripper_power_off_pending:
                self._push_event(
                    'warning',
                    'Gripper 12 V OFF is already in progress.',
                )
                return
            self._gripper_power_off_pending = True

        # Releasing torque before removing power avoids leaving a commanded
        # hold active. If the driver was started without power its service may
        # be unavailable or return failure; power OFF must still proceed.
        if (
            self.gripper_torque_client is None
            or not self.gripper_torque_client.service_is_ready()
        ):
            self._push_event(
                'warning',
                'Gripper torque service is unavailable; continuing with '
                '12 V OFF.',
            )
            with self._lock:
                self._gripper_power_off_pending = False
            self._request_gripper_power_off_now()
            return

        request = SetBool.Request()
        request.data = False
        future = self.gripper_torque_client.call_async(request)
        self._pending_futures.append(future)
        future.add_done_callback(self._gripper_torque_off_before_power_done)
        self._push_event(
            'info',
            'Gripper torque disable requested before 12 V OFF.',
        )

    def _gripper_torque_off_before_power_done(self, future) -> None:
        """Continue power-off sequencing regardless of torque response."""

        self._discard_future(future)
        try:
            result = future.result()
            if result is not None and bool(result.success):
                self._push_event('info', 'Gripper torque disable succeeded.')
            else:
                message = (
                    getattr(result, 'message', 'No response')
                    if result is not None
                    else 'No response'
                )
                self._push_event(
                    'warning',
                    'Gripper torque disable failed; continuing with '
                    f'12 V OFF: {message}',
                )
        except Exception as exc:
            self._push_event(
                'warning',
                'Gripper torque disable call failed; continuing with '
                f'12 V OFF: {exc}',
            )
        finally:
            with self._lock:
                self._gripper_power_off_pending = False

        self._request_gripper_power_off_now()

    def _request_gripper_power_off_now(self) -> None:
        self._request_state_on_off(
            client=self.tool_flange_power_client,
            enabled=False,
            parameters='12v',
            value=0.0,
            label='Gripper 12 V',
        )

    def home_gripper(self) -> None:
        """Initialize and home both grippers only with confirmed 12 V."""

        voltages, fresh, confirmed_12v = self._gripper_power_status()
        if not confirmed_12v:
            detail = (
                'tool-flange voltage feedback is stale or unavailable'
                if not fresh
                else f'measured voltages are {list(voltages or ())} V'
            )
            self._push_event(
                'error',
                'HOME rejected: both tool flanges must report 12 V; '
                f'{detail}.',
            )
            return
        if (
            self.gripper_home_client is None
            or not self.gripper_home_client.service_is_ready()
        ):
            self._push_event(
                'warning',
                f'Gripper HOME service not ready: {self.gripper_home_service}',
            )
            return
        if (
            self._active_gripper_task_id is not None
            or self._gripper_controller.status().motion_active
        ):
            self._push_event(
                'warning',
                'Gripper HOME rejected while a gripper command is active.',
            )
            return
        with self._lock:
            if self._gripper_home_pending:
                self._push_event(
                    'warning',
                    'Gripper HOME is already in progress.',
                )
                return
            self._gripper_home_pending = True

        future = self.gripper_home_client.call_async(Trigger.Request())
        self._pending_futures.append(future)
        future.add_done_callback(self._gripper_home_done)
        self._push_event(
            'warning',
            'Gripper HOME requested; both grippers will traverse their full '
            'stroke.',
        )

    def _gripper_home_done(self, future) -> None:
        self._discard_future(future)
        with self._lock:
            self._gripper_home_pending = False
        try:
            result = future.result()
        except Exception as exc:
            self._push_event('error', f'Gripper HOME call failed: {exc}')
            return
        message = (
            getattr(result, 'message', '')
            if result is not None
            else 'No response'
        )
        if result is not None and bool(result.success):
            suffix = f': {message}' if message else ''
            self._push_event('info', f'Gripper HOME succeeded{suffix}')
        else:
            self._push_event('error', f'Gripper HOME failed: {message}')

    def _control_manager_done(
        self,
        future,
        command: str,
    ) -> None:
        """Process a ControlManagerCommand service response."""

        self._discard_future(future)

        try:
            result = future.result()
        except Exception as exc:
            self._push_event(
                'error',
                'Control Manager '
                f'{command.upper()} call failed: {exc}',
            )
            return

        if result is not None and bool(result.success):
            message = getattr(result, 'message', '')
            suffix = f': {message}' if message else ''
            self._push_event(
                'info',
                'Control Manager '
                f'{command.upper()} succeeded{suffix}',
            )
            return

        message = (
            getattr(result, 'message', 'No response')
            if result is not None
            else 'No response'
        )
        self._push_event(
            'error',
            'Control Manager '
            f'{command.upper()} failed: {message}',
        )

    def _request_state_on_off(
        self,
        client,
        enabled: bool,
        parameters: str,
        value: float,
        label: str,
    ) -> None:
        """Common StateOnOff service request function."""

        if (
            not self.services_enabled
            or client is None
        ):
            self._push_event(
                'warning',
                f'{label} service is unavailable.',
            )
            return

        if not client.service_is_ready():
            self._push_event(
                'warning',
                f'{label} service not ready: '
                f'{client.srv_name}',
            )
            return

        assert StateOnOff is not None

        request = StateOnOff.Request()

        request.state = bool(enabled)
        request.parameters = str(parameters)
        request.value = float(value)

        future = client.call_async(request)

        self._pending_futures.append(future)

        future.add_done_callback(
            lambda done,
            service_label=label,
            requested=bool(enabled):
            self._state_on_off_done(
                done,
                service_label,
                requested,
            )
        )

        self._push_event(
            'info',
            f'{label} '
            f'{"ON" if enabled else "OFF"} '
            f'requested.',
        )

    def _state_on_off_done(
        self,
        future,
        label: str,
        enabled: bool,
    ) -> None:
        """Process Power, Servo, or Stream service response."""

        self._discard_future(future)

        try:
            result = future.result()
        except Exception as exc:
            self._push_event(
                'error',
                f'{label} service call failed: {exc}',
            )
            return

        if (
            result is not None
            and bool(result.success)
        ):
            # Normally stream state is taken from /robot_state.
            # Use the service result only as a fallback when the
            # robot-state topic has never been received.
            if (
                label == 'Stream'
                and not self._robot_state_received
            ):
                self.stream_enabled = enabled

            self._push_event(
                'info',
                f'{label} '
                f'{"ON" if enabled else "OFF"} '
                f'succeeded.',
            )
            return

        message = (
            getattr(result, 'message', 'No response')
            if result is not None
            else 'No response'
        )

        self._push_event(
            'error',
            f'{label} request failed: {message}',
        )

    def _process_prepare_operation(self) -> None:
        """Process the Power -> Servo preparation state machine."""

        if self._prepare_stage == 'idle':
            return

        now = time.monotonic()

        if now > self._prepare_deadline:
            self._push_event(
                'error',
                'Robot preparation timed out.',
            )

            self._prepare_stage = 'idle'
            return

        if self._prepare_stage == 'power_request':
            if (
                self.power_client is None
                or not self.power_client.service_is_ready()
            ):
                return

            self._call_state_service(
                self.power_client,
                state=True,
                parameters='all',
                callback=self._power_done,
            )

            self._prepare_stage = 'power_pending'

            self._push_event(
                'info',
                'Power ON request sent.',
            )

            return

        if self._prepare_stage == 'power_wait':
            if now >= self._prepare_not_before:
                self._prepare_stage = 'servo_request'

            return

        if self._prepare_stage == 'servo_request':
            if (
                self.servo_client is None
                or not self.servo_client.service_is_ready()
            ):
                return

            self._call_state_service(
                self.servo_client,
                state=True,
                parameters='all',
                callback=self._servo_done,
            )

            self._prepare_stage = 'servo_pending'

            self._push_event(
                'info',
                'Servo ON request sent.',
            )

            return

        if self._prepare_stage == 'wait_state':
            if self.control_state in (2, 3):
                self._push_event(
                    'info',
                    'Robot preparation completed.',
                )

                self._prepare_stage = 'idle'

            elif (
                self.control_state is None
                and now >= self._prepare_not_before
            ):
                self._push_event(
                    'warning',
                    'Servo request succeeded, but '
                    'robot_state is unavailable; '
                    'continuing without state confirmation.',
                )

                self._prepare_stage = 'idle'

    def _call_state_service(
        self,
        client,
        state: bool,
        parameters: str,
        callback,
    ) -> None:
        """Internal StateOnOff request used by Prepare Robot."""

        assert StateOnOff is not None

        request = StateOnOff.Request()

        request.state = bool(state)
        request.parameters = str(parameters)
        request.value = 0.0

        future = client.call_async(request)

        self._pending_futures.append(future)

        future.add_done_callback(callback)

    def _power_done(self, future) -> None:
        self._discard_future(future)

        result = self._safe_service_result(
            future,
            'Power ON',
        )

        if result:
            self._prepare_not_before = (
                time.monotonic() + 1.0
            )

            self._prepare_stage = 'power_wait'
        else:
            self._prepare_stage = 'idle'

    def _servo_done(self, future) -> None:
        self._discard_future(future)

        result = self._safe_service_result(
            future,
            'Servo ON',
        )

        if result:
            self._prepare_not_before = (
                time.monotonic() + 1.0
            )

            self._prepare_stage = 'wait_state'
        else:
            self._prepare_stage = 'idle'

    def _safe_service_result(
        self,
        future,
        name: str,
    ) -> bool:
        try:
            result = future.result()
        except Exception as exc:
            self._push_event(
                'error',
                f'{name} service failed: {exc}',
            )

            return False

        if (
            result is not None
            and bool(result.success)
        ):
            self._push_event(
                'info',
                f'{name} succeeded.',
            )

            return True

        message = (
            getattr(result, 'message', 'No response')
            if result is not None
            else 'No response'
        )

        self._push_event(
            'error',
            f'{name} failed: {message}',
        )

        return False

    def _discard_future(self, future) -> None:
        try:
            self._pending_futures.remove(future)
        except ValueError:
            pass

    def snapshot(self) -> BackendSnapshot:
        """Return a thread-safe state snapshot for the Qt UI."""

        command, stale = self._current_command()

        readiness = {
            'power': bool(
                self.power_client
                and self.power_client.service_is_ready()
            ),
            'servo': bool(
                self.servo_client
                and self.servo_client.service_is_ready()
            ),
            'stream': bool(
                self.stream_client
                and self.stream_client.service_is_ready()
            ),
            'control_manager': bool(
                self.control_manager_client
                and self.control_manager_client.service_is_ready()
            ),
            'tool_flange_power': bool(
                self.tool_flange_power_client
                and self.tool_flange_power_client.service_is_ready()
            ),
            'gripper_home': bool(
                self.gripper_home_client
                and self.gripper_home_client.service_is_ready()
            ),
            'gripper_torque': bool(
                self.gripper_torque_client
                and self.gripper_torque_client.service_is_ready()
            ),
            'cartesian_pose': bool(
                self.cartesian_pose_client
                and self.cartesian_pose_client.service_is_ready()
            ),
            'cancel_control': bool(
                self.cancel_control_client
                and self.cancel_control_client.service_is_ready()
            ),
            'joint_action': bool(
                self.joint_action_client
                and self.joint_action_client.server_is_ready()
            ),
            'cartesian_action': bool(
                self.cartesian_action_client
                and self.cartesian_action_client.server_is_ready()
            ),
        }

        now = time.monotonic()
        robot_status = self.robot_session.snapshot()
        power_enabled = robot_status.power_enabled
        servo_enabled = robot_status.servo_enabled
        power_feedback_time = robot_status.power_updated_at
        servo_feedback_time = robot_status.servo_updated_at

        if (
            power_feedback_time is None
            or now - power_feedback_time
            > self.power_servo_feedback_timeout_sec
        ):
            power_enabled = None

        if (
            servo_feedback_time is None
            or now - servo_feedback_time
            > self.power_servo_feedback_timeout_sec
        ):
            servo_enabled = None

        gripper = self._gripper_controller.status()
        gripper_power_voltages, gripper_power_fresh, gripper_power_12v = (
            self._gripper_power_status(now)
        )
        readiness['gripper_driver'] = bool(
            gripper.ready is True and gripper.state_fresh
        )

        return BackendSnapshot(
            namespace=self.get_namespace(),

            cmd_vel_topic=self.mobile_base_controller.cmd_vel_topic,

            cmd_vel_subscribers=(
                self.mobile_base_controller.subscriber_count()
            ),

            control_state=robot_status.control_state,
            power_enabled=power_enabled,
            servo_enabled=servo_enabled,
            stream_enabled=robot_status.stream_enabled,
            emo_active=robot_status.emo_active,
            collision_active=robot_status.collision_active,

            services_enabled=self.services_enabled,
            rby1_msgs_available=RBY1_MSGS_AVAILABLE,

            service_ready=readiness,

            command=command,
            command_stale=stale,
            gripper_ready=gripper.ready,
            gripper_state_fresh=gripper.state_fresh,
            gripper_positions=gripper.positions,
            gripper_target=gripper.target,
            gripper_motion_active=gripper.motion_active,
            gripper_error=gripper.error,
            gripper_power_voltages=gripper_power_voltages,
            gripper_power_state_fresh=gripper_power_fresh,
            gripper_power_12v=gripper_power_12v,
        )

    def shutdown_safely(
        self,
        turn_stream_off: bool = True,
    ) -> None:
        """Stop motion and optionally request Stream OFF."""

        self.stop(publish_immediately=True)
        self.cancel_motion()

        for _ in range(4):
            self._publish_twist(
                VelocityCommand()
            )

        if (
            turn_stream_off
            and self.stream_enabled is True
        ):
            self.request_stream(False)

    def close(self) -> None:
        """Release non-ROS resources owned by this node."""

        self.robot_session.close()
