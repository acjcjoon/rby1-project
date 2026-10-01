"""Validated persistent named poses for the VSLAM operator UI."""

from dataclasses import asdict, dataclass
import math
import os
from pathlib import Path
from typing import Iterable, List

import yaml


def default_waypoints_path(package_share) -> Path:
    """Use source package data with symlink-install, otherwise installed data."""
    share = Path(package_share).resolve()
    # colcon --symlink-install links config files back to the source package.
    source_root = (share / 'config' / 'isaac_vslam.yaml').resolve().parent.parent
    package_root = source_root if (source_root / 'package.xml').is_file() else share
    return package_root / 'data' / 'rby1_vslam_waypoints.yaml'


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('waypoint name must not be empty')
        if any(not math.isfinite(float(value)) for value in (self.x, self.y, self.yaw)):
            raise ValueError('waypoint coordinates must be finite')


def next_name(waypoints: Iterable[Waypoint]) -> str:
    used = {item.name for item in waypoints}
    index = 1
    while f'Point {index}' in used:
        index += 1
    return f'Point {index}'


def load_waypoints(path, frame_id='vslam_map') -> List[Waypoint]:
    target = Path(path).expanduser()
    if not target.exists():
        return []
    with target.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('unsupported waypoint file schema')
    if document.get('frame_id') != frame_id:
        raise ValueError(
            f"waypoint frame {document.get('frame_id')!r} does not match {frame_id!r}"
        )
    raw_items = document.get('waypoints', [])
    if not isinstance(raw_items, list):
        raise ValueError('waypoints must be a list')
    result = []
    names = set()
    for raw in raw_items:
        if not isinstance(raw, dict):
            raise ValueError('each waypoint must be an object')
        item = Waypoint(
            name=str(raw.get('name', '')).strip(),
            x=float(raw.get('x')),
            y=float(raw.get('y')),
            yaw=float(raw.get('yaw')),
        )
        if item.name in names:
            raise ValueError(f'duplicate waypoint name: {item.name}')
        names.add(item.name)
        result.append(item)
    return result


def save_waypoints(path, waypoints: Iterable[Waypoint], frame_id='vslam_map') -> None:
    target = Path(path).expanduser()
    if not target.is_absolute():
        raise ValueError('waypoints_file must be an absolute path')
    items = list(waypoints)
    names = [item.name for item in items]
    if len(names) != len(set(names)):
        raise ValueError('waypoint names must be unique')
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + '.tmp')
    document = {
        'schema_version': 1,
        'frame_id': frame_id,
        'waypoints': [asdict(item) for item in items],
    }
    try:
        with temporary.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
