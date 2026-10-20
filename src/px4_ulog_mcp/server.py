"""MCP server: read-only tools over PX4 ULog flight logs."""

import functools
import os
from typing import Annotated

from mcp.server.mcpserver import Image, MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import Field

from . import logs, report

INSTRUCTIONS = """\
Tools for reading PX4 flight logs (.ulg). Start with log_summary; it answers
most "what happened in this flight" questions (arming, modes, failsafes,
estimator trouble, warnings). Narrow down with flight_timeline, log_messages,
estimator_health, estimate_vs_truth (SITL only), velocity_tracking, log_params,
topic_data and plot_series.

A log is named by path, file name ('16_21_32.ulg'), date/name
('2026-10-20/16_21_32'), 'latest' or 'latest-N'. list_logs with `at` finds the
log that was running at a local wall-clock time. Times in reports are seconds
since the log started. PX4 names folders and files in UTC; `day` arguments and
the start times in reports are local.
"""

server = MCPServer('px4-ulog', instructions=INSTRUCTIONS)

LogRef = Annotated[str, Field(description="path, '16_21_32.ulg', '2026-10-20/16_21_32', 'latest' or 'latest-N'")]
TFrom = Annotated[float | None, Field(description='start, s since log start')]
TTo = Annotated[float | None, Field(description='end, s since log start')]


def tool(fn):
    """Register fn as a tool; a ValueError (bad log name, unknown topic...) reaches
    the caller as a tool error with its message, anything else stays a crash."""
    @functools.wraps(fn)
    def run(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except ValueError as exc:
            raise ToolError(str(exc)) from exc
    server.tool()(run)
    return run


def _open(log, topics=None):
    # every report's header needs the log's span, taken from these two topics
    path = logs.resolve(log)
    return path, logs.load(path, (*topics, 'vehicle_status', 'vehicle_local_position') if topics else None)


@tool
def list_logs(
    at: Annotated[str, Field(description="local time 'HH:MM[:SS]' (today) or 'YYYY-MM-DD HH:MM' to find the log running then")] = '',
    day: Annotated[str, Field(description="only logs started on this local date, 'YYYY-MM-DD'")] = '',
    limit: Annotated[int, Field(ge=1)] = 20,
) -> str:
    """List flight logs newest first with their local start time, or find the one
    that covers a given wall-clock time."""
    if at:
        path = logs.find_at(at)
        if path is None:
            return f'no log was running at {at} (dirs: {logs.log_dirs()})'
        return report.scan_line(path, '')[0] + f'\n{path}'
    paths = [p for p in reversed(logs.all_logs())
             if not day or logs.local(logs.start_time(p)).startswith(day)]
    lines = [f'{len(paths)} logs under {", ".join(logs.log_dirs())}' + (f' on {day}' if day else '')]
    for p in paths[:limit]:
        lines.append(f'{os.path.relpath(p, os.path.dirname(os.path.dirname(p)))}  '
                     f'{logs.local(logs.start_time(p))}  {os.path.getsize(p) / 1e6:.1f} MB')
    if len(paths) > limit:
        lines.append(f'... {len(paths) - limit} more')
    return '\n'.join(lines)


@tool
def log_summary(log: LogRef = 'latest') -> str:
    """What happened in one flight: PX4 version, arming and why, takeoff and landing,
    mode changes, failsafes, estimator trouble while armed, distinct warnings and
    errors (console and decoded events)."""
    path, u = _open(log)
    return report.summary(u, path)


@tool
def log_messages(
    log: LogRef = 'latest',
    min_level: Annotated[str, Field(description='EMERGENCY ALERT CRITICAL ERROR WARNING NOTICE INFO DEBUG')] = 'WARNING',
    contains: Annotated[str, Field(description='case-insensitive text filter')] = '',
    t_from: TFrom = None,
    t_to: TTo = None,
    limit: int = 100,
) -> str:
    """PX4 console messages and decoded events at or above a level, oldest first."""
    path, u = _open(log, ('event',))
    return report.messages_report(u, path, min_level, contains, t_from, t_to, limit)


@tool
def flight_timeline(log: LogRef = 'latest', limit: int = 120) -> str:
    """Everything that changed, in order: modes and arming, commands and their acks,
    failsafe flags set/cleared, takeoff/landing, estimator validity, warnings."""
    path, u = _open(log, ('vehicle_status', 'vehicle_command', 'vehicle_command_ack', 'failsafe_flags',
                          'vehicle_land_detected', 'vehicle_local_position', 'event'))
    return report.timeline(u, path, limit)


@tool
def estimator_health(log: LogRef = 'latest') -> str:
    """EKF2 view: when local position/velocity were invalid, dead reckoning, resets,
    which sources were fused and when (GPS, mag, baro...), faults, and innovation
    test ratios that exceeded 1 (measurements rejected)."""
    path, u = _open(log, ('vehicle_local_position', 'estimator_status_flags',
                          'estimator_innovation_test_ratios'))
    return report.estimator_report(u, path)


@tool
def estimate_vs_truth(log: LogRef = 'latest') -> str:
    """SITL only: PX4's position/yaw estimate against the simulator's ground truth,
    on the ground before liftoff and in the air, plus sudden jumps. Shows offsets
    a planner flying on the estimate would carry."""
    path, u = _open(log, ('vehicle_local_position', 'vehicle_local_position_groundtruth',
                          'vehicle_attitude', 'vehicle_attitude_groundtruth'))
    return report.truth_report(u, path)


@tool
def velocity_tracking(log: LogRef = 'latest') -> str:
    """How PX4 followed offboard velocity setpoints: fits a first-order lag plus dead
    time from trajectory_setpoint to the measured velocity (the 'reaction time' an
    avoidance planner has to budget for)."""
    path, u = _open(log, ('trajectory_setpoint', 'vehicle_local_position', 'vehicle_status'))
    return report.tracking_report(u, path)


@tool
def log_params(
    log: LogRef = 'latest',
    names: Annotated[list[str], Field(description='exact parameter names')] = [],
    pattern: Annotated[str, Field(description="glob over names, e.g. 'EKF2_*' or '*_DLL_*'")] = '',
) -> str:
    """Parameter values as logged at boot, and any changed during the log."""
    path, u = _open(log, ('parameter_update',))
    return report.params_report(u, path, names, pattern)


@tool
def list_topics(log: LogRef = 'latest') -> str:
    """Every logged uORB topic with sample count, rate and field count."""
    path, u = _open(log)
    return report.topics_report(u, path)


@tool
def topic_data(
    topic: Annotated[str, Field(description="uORB topic, e.g. 'vehicle_local_position' or 'sensor_mag[1]'")],
    log: LogRef = 'latest',
    fields: Annotated[list[str], Field(description='fields to report; empty = all')] = [],
    t_from: TFrom = None,
    t_to: TTo = None,
    points: Annotated[int, Field(description='also return this many evenly spaced samples (0 = stats only)')] = 0,
) -> str:
    """Min/mean/max/first/last of a topic's fields over a time window, optionally
    with a downsampled table."""
    path, u = _open(log, (topic.partition('[')[0],))
    return report.topic_data_report(u, path, topic, fields, t_from, t_to, points)


@tool
def scan_logs(
    day: Annotated[str, Field(description="local date 'YYYY-MM-DD'; empty = the latest `limit` logs")] = '',
    contains: Annotated[str, Field(description='only count warnings containing this text')] = '',
    limit: Annotated[int, Field(ge=1, description='at most this many of the newest logs')] = 40,
) -> str:
    """One line per log: start time, duration, time armed, first failsafe, warning
    count and the first warning. For questions like 'which flights today lost the
    compass' or 'which ones failsafed'."""
    paths = [p for p in logs.all_logs() if not day or logs.local(logs.start_time(p)).startswith(day)]
    total = len(paths)
    paths = paths[-limit:]
    rows = []
    for p in paths:
        try:
            line, n = report.scan_line(p, contains)
        except ValueError as exc:                    # one bad file must not hide the rest
            line, n = f'{os.path.basename(p)}  skipped: {exc}', 1
        if n or not contains:
            rows.append(line)
    head = (f'{len(paths)} of {total} logs' + (f' on {day}' if day else '') + ' scanned'
            + (f"; {len(rows)} with warnings containing '{contains}'" if contains else ''))
    return '\n'.join([head] + rows)


@tool
def plot_series(
    series: Annotated[list[str], Field(description="one subplot per entry; 'topic.field', or several joined with '+', "
                                                   "e.g. 'vehicle_local_position.z+vehicle_local_position_groundtruth.z'")],
    log: LogRef = 'latest',
    t_from: TFrom = None,
    t_to: TTo = None,
) -> Image:
    """Plot logged fields against time (mode changes dashed, failsafes red) and
    return the PNG."""
    path, u = _open(log)
    return Image(data=report.plot_png(u, path, series, t_from, t_to), format='png')


def main():
    server.run('stdio')
