"""Parser for the terminal gripper debug controller."""

from __future__ import annotations

from dataclasses import dataclass
import math
import shlex
from typing import Optional, Tuple


@dataclass(frozen=True)
class DebugInstruction:
    action: str
    side: Optional[str] = None
    values: Tuple[float, ...] = ()
    enabled: Optional[bool] = None


def _ratio(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0 or result > 1.0:
        raise ValueError('close ratio must be within [0.0, 1.0]')
    return result


def parse_debug_command(line: str) -> DebugInstruction:
    """Parse one interactive command without depending on ROS."""

    tokens = shlex.split(line)
    if not tokens:
        return DebugInstruction('noop')
    command = tokens[0].lower()

    if command in ('help', '?') and len(tokens) == 1:
        return DebugInstruction('help')
    if command in ('quit', 'exit', 'q') and len(tokens) == 1:
        return DebugInstruction('quit')
    if command in ('state', 'status') and len(tokens) == 1:
        return DebugInstruction('state')
    if command == 'home' and len(tokens) == 1:
        return DebugInstruction('home')
    if command == 'torque' and len(tokens) == 2:
        setting = tokens[1].lower()
        if setting not in ('on', 'off'):
            raise ValueError('usage: torque <on|off>')
        return DebugInstruction('torque', enabled=(setting == 'on'))
    if command == 'set' and len(tokens) == 3:
        return DebugInstruction(
            'set', values=(_ratio(tokens[1]), _ratio(tokens[2]))
        )
    if command in ('right', 'left') and len(tokens) == 2:
        return DebugInstruction(
            'set_side', side=command, values=(_ratio(tokens[1]),)
        )
    if command in ('open', 'close') and len(tokens) in (1, 2):
        side = tokens[1].lower() if len(tokens) == 2 else 'both'
        if side not in ('right', 'left', 'both'):
            raise ValueError(
                f'usage: {command} [right|left|both]'
            )
        return DebugInstruction(
            'preset',
            side=side,
            values=(0.0 if command == 'open' else 1.0,),
        )
    raise ValueError('unknown command or wrong arguments; enter help')
