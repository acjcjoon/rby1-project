import math

import pytest

from rby1_navigation.se2_utils import (
    body_frame_error,
    compose,
    proportional_velocity,
)


def test_relative_target_is_fixed_using_pose_at_command_receipt():
    start = (2.0, 3.0, math.pi / 2.0)

    target = compose(start, (1.0, -0.5, math.pi / 4.0))

    assert target == pytest.approx((2.5, 4.0, 3.0 * math.pi / 4.0))
    assert body_frame_error(start, target) == pytest.approx(
        (1.0, -0.5, math.pi / 4.0)
    )


def test_proportional_velocity_preserves_direction_while_limiting_speed():
    command = proportional_velocity(
        (0.0, 0.0, 0.0),
        (3.0, 4.0, 1.0),
        linear_kp=1.0,
        angular_kp=2.0,
        max_linear=0.5,
        max_angular=0.3,
    )

    assert command == pytest.approx((0.3, 0.4, 0.3))
