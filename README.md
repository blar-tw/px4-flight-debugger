# px4-flight-debugger

Is it my planner, or is it PX4? An MCP server for people writing offboard and
autonomy code against PX4 SITL: Claude (or any MCP client) reads the `.ulg`
and answers "why did that flight never reach its goal?", "how far off was the
estimate from the simulator's truth?" or "how late does PX4 follow my velocity
commands?" in one call instead of a hand-written pyulog script.

[![tests](https://github.com/blar-tw/px4-flight-debugger/actions/workflows/ci.yml/badge.svg)](https://github.com/blar-tw/px4-flight-debugger/actions/workflows/ci.yml)

```
> log_summary 2026-10-20/16_21_32

log 2026-10-20/16_21_32.ulg  started 2026-10-20 09:21:32 PDT, 50.9 s, 10.2 MB
PX4 v1.14.4 dev on PX4_SITL
armed: 12.6s (internal command) -> 49.9s (landing)
in the air 14.4-47.9s
[...]
modes: 0.0 AUTO_LOITER disarmed -> 12.6 OFFBOARD armed -> 17.6 ALTCTL FAILSAFE -> 49.9 OFFBOARD disarmed
failsafe: first at 17.6s
estimator while armed: horizontal position invalid 17.5-50.0s; dead reckoning 13.5-50.0s
warnings and errors (6 distinct):
    13.8s WARNING  [health_and_arming_checks] Preflight Fail: No valid data from Compass 0  (x3, last 22.6s)
    17.5s WARNING  [mc_pos_control] Failsafe: blind land  (x15, last 46.1s)
    [...]
    17.6s WARNING  Failsafe activated, triggering fallback to altitude control
    [...]
```

That flight came from an obstacle-avoidance batch in Gazebo where, in two runs
out of sixteen, the planner never got to start navigating. The planner was
fine: SITL lost its compass right after arming, the estimator lost its
horizontal position four seconds later and PX4 failsafed into a blind landing.

## Why

Developing offboard planners against PX4 SITL, most of the "is it my code or
the simulator?" questions are answered by the flight log, but reading one
means a fresh script each time: find which `.ulg` belongs to the flight, decode
the mode numbers, line up messages, events and estimator flags. The tools here
are the questions that kept coming up, answered in plain text an assistant can
read and reason about.

## Tools

| Tool | Answers |
|---|---|
| `list_logs` | which logs exist; which one was running at 10:07 local time |
| `log_summary` | what happened: version, arming and why, takeoff/landing, modes, failsafes, estimator trouble, distinct warnings |
| `flight_timeline` | everything that changed, in order: modes, commands and acks, failsafe flags, takeoff, estimator validity, warnings |
| `log_messages` | console messages and decoded events by level, text and time window |
| `estimator_health` | when position/velocity were invalid, dead reckoning, resets, which sources were fused when, faults, rejected measurements |
| `estimate_vs_truth` | SITL only: estimate against the simulator's ground truth, on the ground before liftoff and in the air |
| `velocity_tracking` | PX4's response to offboard velocity setpoints as a first-order lag plus dead time |
| `log_params` | parameter values by name or glob, and changes during the flight |
| `list_topics`, `topic_data` | any uORB topic: fields, rates, stats over a window, a downsampled table |
| `scan_logs` | one line per log for a whole day: which ones failsafed, which ones mention "compass" |
| `plot_series` | a PNG of any fields against time, mode changes marked |

All read-only. A log is named by path, file name (`16_21_32.ulg`), date/name
(`2026-10-20/16_21_32`), `latest` or `latest-N`. Times are seconds since the
log started; the local wall-clock start is in every header. PX4 names log
folders and files in UTC, while `day` arguments and printed start times are
local.

## Install

Needs [uv](https://docs.astral.sh/uv/).

```bash
git clone https://github.com/blar-tw/px4-flight-debugger && cd px4-flight-debugger
uv sync
```

Register it with Claude Code (user scope, so every project sees it):

```bash
claude mcp add px4-flight-debugger --scope user -- uv --directory /path/to/px4-flight-debugger run px4-flight-debugger
```

or add it to an MCP client's config by hand:

```json
{
  "mcpServers": {
    "px4-flight-debugger": {
      "command": "uv",
      "args": ["--directory", "/path/to/px4-flight-debugger", "run", "px4-flight-debugger"],
      "env": {"PX4_ULOG_DIRS": "/home/you/PX4-Autopilot/build/px4_sitl_default/rootfs/log", "PYTHONPATH": ""}
    }
  }
}
```

`PX4_ULOG_DIRS` lists where to look (separated by `:`), default
`~/PX4-Autopilot/build/px4_sitl_default/rootfs/log`. Empty `PYTHONPATH` keeps a
sourced ROS 2 environment from leaking its packages into the server.

## How it reads a log

- [pyulog](https://github.com/PX4/pyulog) parses the file; each tool asks only
  for the topics it needs, and a few parsed logs are kept in memory, keyed on
  the file's modification time.
- Events are decoded with the definitions PX4 embeds in the log, falling back
  to a local `all_events.json` from a PX4 build. Nothing is downloaded.
- Mode, arming and command names are PX4 v1.14's. The arming reason names come
  from commander's `arm_disarm_reason_t`, not the `ARM_DISARM_REASON_*`
  constants in `VehicleStatus.msg`, which are out of date (6 is "landing", not
  "safety button").
- `velocity_tracking` only counts setpoints with position NaN and velocity set
  while PX4 is in offboard, which is what it flies as a velocity command. A
  position setpoint whose velocity fields were simply left at zero is not one;
  counting those halved the fitted lag before this rule. It says so when the
  fit does not look like a lagged velocity follower.
- `estimate_vs_truth` takes the truth relative to where the drone spawned (the
  simulator's world frame need not start at PX4's local origin) and reports
  separately how far PX4 placed its local origin from the simulator's.
- pyulog prints warnings on stdout, which is the protocol channel on stdio;
  they are sent to stderr instead. An unreadable file is a tool error, and
  `scan_logs` skips it with the reason.

## Things the logs showed while building it

- **SITL logs carry the ground truth.** `vehicle_local_position_groundtruth`
  and `vehicle_attitude_groundtruth` are logged next to the estimate, so
  estimate-vs-truth needs no Gazebo bridge or extra recorder. Across 97 SITL
  flights here, the height estimate had settled between 1.17 m below and 1.22 m
  above the truth while the drone still sat on the ground (either sign, more
  than 0.3 m in 60 of them), and the offset carried into the air. Horizontally,
  most of the error is where PX4 put its local origin at the first GPS fix: in
  17_06_45 the origin sits 0.35 m from the spawn point and the estimate is off
  by 0.39 m.
- **PX4's own lag and the planner's.** Over 87 flights with a clean fit, PX4
  followed an offboard velocity setpoint like a first-order lag of 0.45 s
  (0.40-0.55) after 0.20 s of dead time. For 17_06_45 the log says 0.65 s in
  total, while the planner that sent the setpoints measured about 0.75 s from
  its own side: transport and scheduling add roughly 0.1 s, and the end-to-end
  figure is the one a planner should budget.
- **A message mismatch, visible as an arming reason.** The planner arms through
  a `VehicleCommand` with `from_external` set, yet the log says "internal
  command". Its `px4_msgs` (main) has `source_component` as `uint16` where
  v1.14 has `uint8`, which shifts the fields after it.

## Limits

- Built and tested against PX4 v1.14 SITL logs. Other versions parse, but
  enum values they added show up as numbers.
- `estimate_vs_truth` needs a SITL log; real flights have no truth topic. It
  takes the first truth sample as the spawn point, so the drone should be at
  rest when logging starts (PX4 starts logging at boot by default).
- PX4's default logging profile thins `trajectory_setpoint` to about 5 Hz, so
  the fitted dead time is only good to about 0.2 s.

## Development

```bash
uv run pytest
```

The tests run on two real SITL logs cut down with
`tools/make_fixture.py` (1.3 MB together): the compass failure above and a
clean offboard flight whose height estimate drifts. Each test is a question
that once needed its own script, plus an MCP round trip in-process and over a
real stdio subprocess.

MIT licensed.
