"""Map CLI validation and completion semantics (no ROS installation needed)."""

import math

import pytest

from rby1_vslam.map_tool import (
    CompletionLog, build_parser, pose_quaternion, validate_path,
)


def test_localization_never_accepts_old_or_unrelated_completion():
    state = CompletionLog("localize", "/maps/room", "visual_slam", since_ns=100)
    state.observe("visual_slam", "Localization completed successfully", 101)
    assert not state.done
    state.observe("visual_slam", "Starting async localization in map '/maps/room'", 99)
    state.observe("visual_slam", "Localization completed successfully", 101)
    assert not state.done
    state.observe("other_visual_slam", "Starting async localization in map '/maps/room'", 102)
    assert not state.started
    state.observe("visual_slam", "Starting async localization in map '/maps/wrong'", 102)
    assert not state.started
    state.observe("visual_slam", "Starting async localization in map '/maps/room'", 102)
    state.observe("visual_slam", "Localization completed successfully", 103)
    assert state.done


def test_save_failure_is_not_mistaken_for_service_acceptance():
    state = CompletionLog("save", "/maps/room", "visual_slam", since_ns=100)
    state.observe("visual_slam", "Saving map to /maps/room", 101)
    state.observe("visual_slam", "Failed to save map", 102)
    assert state.error == "Failed to save map"
    assert not state.done


def test_localization_failure_is_reported():
    state = CompletionLog("localize", "/maps/room", "visual_slam", since_ns=100)
    state.observe("visual_slam", "Starting async localization in map '/maps/room'", 101)
    state.observe("visual_slam", "Failed to localize in map. Error not found", 102)
    assert state.error
    assert not state.done


def test_map_path_requires_real_database_for_localize(tmp_path):
    with pytest.raises(ValueError, match="absolute"):
        validate_path("relative/map", "save")
    with pytest.raises(ValueError, match=".mdb"):
        validate_path(str(tmp_path), "localize")
    (tmp_path / "map.mdb").write_bytes(b"map")
    assert validate_path(str(tmp_path), "localize") == str(tmp_path)
    assert validate_path(str(tmp_path / "new_map"), "save").endswith("new_map")


def test_pose_hint_is_unit_quaternion():
    q = pose_quaternion(0.3, -0.2, math.pi / 2)
    assert sum(value * value for value in q) == pytest.approx(1.0)
    assert pose_quaternion(0, 0, 0) == (0, 0, 0, 1)


@pytest.mark.parametrize("option,value", [("--yaw", "nan"), ("--x", "inf"),
                                          ("--timeout", "0")])
def test_rejects_invalid_cli_values(option, value):
    with pytest.raises(SystemExit):
        build_parser().parse_args(["localize", "--path", "/maps/room", option, value])
