"""Pure helpers for deterministic VSLAM sensor replay experiments."""


CONDITION_PROFILES = {
    'baseline': ('replay', 'replay'),
    'no_imu': ('replay', 'none'),
    'fixed_stereo': ('freeze_after_marker', 'replay'),
    'fixed_stereo_no_imu': ('freeze_after_marker', 'none'),
}


def condition_profile(name):
    """Return ``(stereo_mode, imu_mode)`` for a named experiment."""
    try:
        return CONDITION_PROFILES[name]
    except KeyError as exc:
        raise ValueError(
            f'unknown condition {name!r}; expected one of '
            f'{", ".join(CONDITION_PROFILES)}') from exc


def rebase_stamp_ns(source_ns, first_source_ns, first_output_ns):
    """Move a recorded timestamp onto the current clock without changing deltas."""
    values = (source_ns, first_source_ns, first_output_ns)
    if any(not isinstance(value, int) or value < 0 for value in values):
        raise ValueError('timestamps must be non-negative integers')
    return first_output_ns + source_ns - first_source_ns
