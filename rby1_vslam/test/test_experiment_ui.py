"""Offscreen walk-through of the station round-trip experiment window."""
import csv
import json
import math
import os
from types import SimpleNamespace

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
QtWidgets = pytest.importorskip('PyQt5.QtWidgets')

from rby1_vslam.experiment_ui import ExperimentWindow  # noqa: E402
from rby1_vslam.waypoint_store import Waypoint  # noqa: E402

STATIONS = [Waypoint('A', 0.0, 0.0, 0.0), Waypoint('B', 1.0, 0.0, 0.0), Waypoint('C', 1.0, 1.0, 1.57)]


class FakeNode:
    def __init__(self, folder):
        self.p = {'experiment_dir': str(folder), 'tag_image_topic': 'img', 'tag_info_topic': 'info'}
        self.base_frame = 'base'
        self.waypoint_path = folder / 'stations.yaml'
        self.correct_on_arrival = True
        self.observer = None
        self.pending = None
        self.active_request = None
        self.goals = []
        self.events = []
        self.experiment_pub = SimpleNamespace(publish=lambda m: self.events.append(json.loads(m.data)))
        self.set_pose(0.0, 0.0, 0.0)

    def set_pose(self, x, y, yaw):
        values = (x, y, 0.0, 0.0, 0.0, yaw)
        self.poses = {'Localization': (None, values, 0.0), 'SLAM': (None, values, 0.0)}

    def pose_fresh(self, name):
        return name in self.poses

    def status(self, name):
        return {'initialized': True, 'healthy': True}

    def fresh(self, name):
        return True

    def emit_log(self, text):
        pass

    def stop(self):
        self.pending = None
        self.active_request = None

    def enable_then_navigate(self, waypoint):
        self.goals.append(waypoint.name)
        self.active_request = len(self.goals)
        return True

    def arrive(self, waypoint, offset=(0.0, 0.0, 0.0)):
        """Simulate Nav2 success at waypoint (+ estimate offset)."""
        self.active_request = None
        self.set_pose(waypoint.x + offset[0], waypoint.y + offset[1], waypoint.yaw + offset[2])
        self.observer.on_arrived(waypoint.name)
        if self.correct_on_arrival:
            self.pending = {'kind': 'station'}
            self.set_pose(waypoint.x, waypoint.y, waypoint.yaw)   # correction snaps to target
            self.pending = None
        self.observer.on_ready(waypoint.name)


@pytest.fixture(scope='module')
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def make(app, tmp_path, condition):
    node = FakeNode(tmp_path)
    win = ExperimentWindow(node, lambda: STATIONS)
    win.show()
    win.rounds_spin.setValue(1)
    win.condition_combo.setCurrentIndex(win.condition_combo.findData(condition))
    win.mode_combo.setCurrentIndex(win.mode_combo.findData('ruler'))
    return node, win


def measure(win, values):
    for key, value in zip(('dx_left_mm', 'dy_left_mm', 'dx_right_mm', 'dy_right_mm'), values):
        win.inputs[key].setText(str(value))
    win.record()


def by_name(name):
    return next(w for w in STATIONS if w.name == name)


def test_vslam_round_records_three_legs(app, tmp_path):
    node, win = make(app, tmp_path, 'vslam')
    win.start()
    assert node.correct_on_arrival is True and node.goals == ['B'] and win.state == 'travel'
    node.arrive(by_name('B'), offset=(0.012, 0.0, 0.0))
    assert win.state == 'measure'
    measure(win, (5, 0, 5, 0))
    assert node.goals == ['B', 'C']
    node.arrive(by_name('C'))
    measure(win, (30, 0, 30, 0))
    node.arrive(by_name('A'))
    win.record(skip=True)
    assert win.state == 'done' and node.correct_on_arrival is True
    rows = list(csv.DictReader(open(win.csv_path)))
    assert [r['to'] for r in rows] == ['B', 'C', 'A']
    assert float(rows[0]['correction_mm']) == pytest.approx(12.0, abs=0.01)
    assert rows[0]['success'] == 'True' and rows[1]['success'] == 'False'
    assert rows[2]['meas_err_mm'] == ''
    assert [e['event'] for e in node.events][:4] == ['start', 'leg_start', 'arrived', 'ready']


def test_wheel_condition_disables_correction_and_blocks_far_start(app, tmp_path):
    node, win = make(app, tmp_path, 'wheel')
    node.set_pose(0.5, 0.0, 0.0)
    win.start()
    assert win.state == 'idle' and node.goals == []          # not at A -> refused
    node.set_pose(0.0, 0.0, 0.0)
    win.start()
    assert node.correct_on_arrival is False
    node.arrive(by_name('B'), offset=(0.0, 0.02, 0.0))
    measure(win, (0, 20, 0, 20))
    row = list(csv.DictReader(open(win.csv_path)))[0]
    assert row['correction_wait_s'] == '' and row['correction_mm'] == ''
    assert float(row['est_err_y_mm']) == pytest.approx(20.0)
    win.abort()
    assert node.correct_on_arrival is True


def test_operator_stop_interrupts_and_retry_resends(app, tmp_path):
    node, win = make(app, tmp_path, 'vslam')
    win.start()
    node.stop()                         # operator pressed STOP while travelling
    win.idle_since = -1e9               # pretend the grace period elapsed
    win.tick()
    assert win.state == 'interrupted'
    win.retry()
    assert node.goals == ['B', 'B'] and win.state == 'travel'


def test_pause_after_current_leg(app, tmp_path):
    node, win = make(app, tmp_path, 'vslam')
    win.start()
    win.request_pause()
    node.arrive(by_name('B'))
    measure(win, (0, 0, 0, 0))
    assert win.state == 'paused' and node.goals == ['B']
    win.resume()
    assert node.goals == ['B', 'C']


def test_window_refuses_to_close_mid_run(app, tmp_path):
    node, win = make(app, tmp_path, 'vslam')
    win.start()
    assert win.close() is False
    win.abort()
    assert win.close() is True


# ---------- AprilTag mode with a fake probe (projected corners, no images) ----------
import numpy as np  # noqa: E402

from rby1_vslam import station_tag as st  # noqa: E402

K = np.array([[385.0, 0, 320.0], [0, 385.0, 240.0], [0, 0, 1]])
T_BASE_CAM = np.array([[0, -0.342, 0.940, 0.05], [-1, 0, 0, 0], [0, -0.940, -0.342, 1.4], [0, 0, 0, 1.0]])
TAG_SIZE = 0.22


def _tag_world(cx, cy, cz):
    h = TAG_SIZE / 2      # tag facing -x (towards the robot), corners TL TR BR BL as seen
    return np.array([[cx, cy + h, cz + h], [cx, cy - h, cz + h], [cx, cy - h, cz - h], [cx, cy + h, cz - h]])


STATION_TAGS = {20: _tag_world(1.0, 0.3, 0.9), 21: _tag_world(1.0, -0.3, 0.9), 22: _tag_world(2.2, 0.0, 1.2)}


class FakeProbe:
    """Returns corners of tags fixed in the *station* frame for the current base offset."""
    def __init__(self):
        self.offset = (0.0, 0.0, 0.0)      # true base pose in the station frame (m, m, deg)
        self.busy = False
        self.pending = None

    def capture(self, callback, duration=2.0, max_frames=40):
        x, y, yaw = self.offset
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        T_station_base = np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        T_cam_station = st.invert(T_station_base @ T_BASE_CAM)
        corners = {}
        for i, P in STATION_TAGS.items():
            pc = (T_cam_station @ np.c_[P, np.ones(4)].T).T[:, :3]
            uv = (K @ pc.T).T
            corners[i] = uv[:, :2] / uv[:, 2:3]
        self.pending = (callback, {'corners': corners, 'corner_sd_px': {i: 0.05 for i in corners},
                                   'detections': {i: 20 for i in corners}, 'frames': 20, 'K': K,
                                   'D': np.zeros(5), 'frame': 'ir', 'T_base_cam': T_BASE_CAM})

    def tick(self):
        if self.pending:
            callback, result = self.pending
            self.pending = None
            callback(result, None)


def make_tag(app, tmp_path, mode='tag'):
    node = FakeNode(tmp_path)
    probe = FakeProbe()
    win = ExperimentWindow(node, lambda: STATIONS, probe=probe)
    win.show()
    win.rounds_spin.setValue(1)
    win.condition_combo.setCurrentIndex(win.condition_combo.findData('vslam'))
    win.mode_combo.setCurrentIndex(win.mode_combo.findData(mode))
    win.settle_spin.setValue(0.0)
    win.tag_size_spin.setValue(TAG_SIZE)
    for name in ('A', 'B', 'C'):
        win.tag_station_combo.setCurrentText(name)
        node.set_pose(by_name(name).x, by_name(name).y, by_name(name).yaw)
        win.register_tags()
        probe.tick()
    node.set_pose(0.0, 0.0, 0.0)
    return node, probe, win


def pump(win, n=3):
    for _ in range(n):
        win.tick()


def test_tag_mode_records_and_advances_automatically(app, tmp_path):
    node, probe, win = make_tag(app, tmp_path)
    assert set(win.tag_refs) == {'A', 'B', 'C'} and win.tag_refs['A']['ids'] == [20, 21, 22]
    assert win.tag_refs_path.exists()
    win.start()
    assert win.state == 'travel'
    probe.offset = (0.012, -0.005, 0.8)        # true arrival error at B
    node.arrive(by_name('B'))
    pump(win)
    assert node.goals == ['B', 'C']             # recorded and moved on without operator input
    row = list(csv.DictReader(open(win.csv_path)))[0]
    assert row['gt_source'] == 'tag'
    assert float(row['gt_x_mm']) == pytest.approx(12.0, abs=0.5)
    assert float(row['gt_y_mm']) == pytest.approx(-5.0, abs=0.5)
    assert float(row['gt_yaw_deg']) == pytest.approx(0.8, abs=0.02)
    assert row['tag_ids'] == '20 21 22' and row['success'] == 'True'


def test_tag_failure_falls_back_to_ruler(app, tmp_path):
    node, probe, win = make_tag(app, tmp_path)
    win.start()
    orig = probe.capture
    probe.capture = lambda cb, **kw: cb(None, 'no IR image')
    node.arrive(by_name('B'))
    pump(win)
    assert win.state == 'measure'
    measure(win, (25, 0, 25, 0))
    row = list(csv.DictReader(open(win.csv_path)))[0]
    assert row['gt_source'] == 'ruler' and row['success'] == 'False' and '[태그 실패]' in row['note']
    probe.capture = orig


def test_tag_mode_requires_references(app, tmp_path):
    node = FakeNode(tmp_path)
    win = ExperimentWindow(node, lambda: STATIONS, probe=FakeProbe())
    win.show()
    win.mode_combo.setCurrentIndex(win.mode_combo.findData('tag'))
    win.start()
    assert win.state == 'idle' and node.goals == []
