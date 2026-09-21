import pytest

from rby1_gripper_driver.debug_commands import parse_debug_command


def test_parse_atomic_set_command():
    command = parse_debug_command('set 0.25 0.75')
    assert command.action == 'set'
    assert command.values == (0.25, 0.75)


@pytest.mark.parametrize(
    ('text', 'side', 'ratio'),
    [
        ('open', 'both', 0.0),
        ('close right', 'right', 1.0),
        ('open left', 'left', 0.0),
    ],
)
def test_parse_open_close_presets(text, side, ratio):
    command = parse_debug_command(text)
    assert command.action == 'preset'
    assert command.side == side
    assert command.values == (ratio,)


def test_parse_rejects_out_of_range_ratio():
    with pytest.raises(ValueError, match=r'\[0.0, 1.0\]'):
        parse_debug_command('right 1.1')


def test_parse_torque_and_quit():
    assert parse_debug_command('torque off').enabled is False
    assert parse_debug_command('quit').action == 'quit'
