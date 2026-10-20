"""Plain-text reports on one parsed ULog. Times are seconds since the log started."""

import fnmatch
import io
import math
import os
import re

import numpy as np

from . import logs
from .names import (ARM_REASON, ARMED, ARMING_STATE, CMD_RESULT, NAV_STATE, VEHICLE_CMD,
                    name, severity)


# --- access helpers ---

def topic(u, topic_name, multi=0):
    for d in u.data_list:
        if d.name == topic_name and d.multi_id == multi:
            return d.data
    return None


def secs(u, ts):
    return (np.asarray(ts, dtype=float) - u.start_timestamp) / 1e6


def duration(u):
    return (u.last_timestamp - u.start_timestamp) / 1e6


def intervals(t, mask):
    """(start, end) of each run where mask is true; end is the first false sample."""
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return []
    edges = np.flatnonzero(np.diff(mask.astype(int)))
    starts = [0] if mask[0] else []
    starts += [i + 1 for i in edges if not mask[i]]
    ends = [i + 1 for i in edges if mask[i]] + ([len(mask) - 1] if mask[-1] else [])
    return [(t[a], t[b]) for a, b in zip(starts, ends)]


def spans(iv):
    return ', '.join(f'{max(a, 0):.1f}-{b:.1f}s' for a, b in iv) or 'never'


def changes(t, values):
    """(t, value) whenever the value differs from the previous sample."""
    values = np.asarray(values)
    keep = np.r_[True, values[1:] != values[:-1]]
    return list(zip(t[keep], values[keep]))


def px4_version(info):
    """'v1.14.4' from ver_sw_release (0xMMmmpptt; tt 255 = release), else the git hash."""
    v = info.get('ver_sw_release')
    if v:
        major, minor, patch, kind = (v >> 24) & 0xff, (v >> 16) & 0xff, (v >> 8) & 0xff, v & 0xff
        tag = {0: ' dev', 64: ' alpha', 128: ' beta', 192: ' rc', 255: ''}.get(kind, f' type {kind}')
        return f'v{major}.{minor}.{patch}{tag}'
    return str(info.get('ver_sw', ''))[:10]


def header(u, path):
    start = logs.start_time(path)
    size = os.path.getsize(path) / 1e6
    info = u.msg_info_dict
    version = px4_version(info)
    return (f"log {os.path.join(os.path.basename(os.path.dirname(path)), os.path.basename(path))}"
            f"  started {logs.local(start)}, {duration(u):.1f} s, {size:.1f} MB"
            + (f"\nPX4 {version} on {info.get('ver_hw', '?')}" if version else ''))


# --- messages ---

def messages(u, min_level='WARNING', contains='', t_from=None, t_to=None):
    """Console messages and decoded events, oldest first, as (t, level, source, text)."""
    out = [(secs(u, m.timestamp).item(), m.log_level_str(), 'console', m.message.strip())
           for m in u.logged_messages]
    out += [(secs(u, ts).item(), lvl, 'event', text) for ts, lvl, text in logs.events(u)]
    worst = severity(min_level)
    pattern = re.compile(re.escape(contains), re.I) if contains else None
    out = [m for m in out if severity(m[1]) <= worst
           and (pattern is None or pattern.search(m[3]))
           and (t_from is None or m[0] >= t_from) and (t_to is None or m[0] <= t_to)]
    return sorted(out)


def dedup(msgs):
    """Collapse repeats: (first t, level, text, count, last t)."""
    seen = {}
    for t, lvl, _src, text in msgs:
        key = re.sub(r'\d+(\.\d+)?', '#', text)
        if key in seen:
            first, l0, txt, n, _ = seen[key]
            seen[key] = (first, l0, txt, n + 1, t)
        else:
            seen[key] = (t, lvl, text, 1, t)
    return sorted(seen.values())


def messages_report(u, path, min_level, contains, t_from, t_to, limit):
    msgs = messages(u, min_level, contains, t_from, t_to)
    lines = [header(u, path), f'{len(msgs)} messages at {min_level.upper()} or worse'
             + (f" containing '{contains}'" if contains else '')]
    for t, lvl, src, text in msgs[:limit]:
        lines.append(f'{t:7.1f}s {lvl:8s} {text}' + ('  [event]' if src == 'event' else ''))
    if len(msgs) > limit:
        lines.append(f'... {len(msgs) - limit} more (raise limit or narrow t_from/t_to)')
    return '\n'.join(lines)


# --- arming, modes, flight ---

def armed_intervals(u):
    st = topic(u, 'vehicle_status')
    if st is None:
        return []
    return intervals(secs(u, st['timestamp']), st['arming_state'] == ARMED)


def mode_changes(u):
    st = topic(u, 'vehicle_status')
    if st is None:
        return []
    t = secs(u, st['timestamp'])
    key = st['nav_state'].astype(int) * 1000 + st['arming_state'].astype(int) * 10 \
        + st['failsafe'].astype(int)
    return [(tt, name(NAV_STATE, k // 1000), name(ARMING_STATE, (k // 10) % 100), bool(k % 10))
            for tt, k in changes(t, key)]


def flight_facts(u):
    """Takeoff/landing times, height gained, distance, peak speed."""
    facts = []
    land = topic(u, 'vehicle_land_detected')
    if land is not None:
        t = secs(u, land['timestamp'])
        air = intervals(t, ~land['landed'].astype(bool))
        if air:
            facts.append('in the air ' + spans(air))
    lp = topic(u, 'vehicle_local_position')
    if lp is not None and len(lp['z']):
        z0 = lp['z'][0]
        steps = np.hypot(np.diff(lp['x']), np.diff(lp['y']))
        facts.append(f"highest {z0 - lp['z'].min():.2f} m above the log's first estimate, "
                     f"{np.hypot(lp['x'] - lp['x'][0], lp['y'] - lp['y'][0]).max():.1f} m from start, "
                     f"path {steps[steps < 1.0].sum():.1f} m, "
                     f"peak speed {np.hypot(lp['vx'], lp['vy']).max():.1f} m/s")
    return facts


def summary(u, path):
    lines = [header(u, path)]
    armed = armed_intervals(u)
    st = topic(u, 'vehicle_status')
    if st is not None:
        t = secs(u, st['timestamp'])
        why = []
        for a, b in armed:
            i = np.searchsorted(t, a)
            j = min(np.searchsorted(t, b), len(t) - 1)
            start = f"{a:.1f}s ({ARM_REASON.get(int(st['latest_arming_reason'][i]), '?')})"
            if st['arming_state'][j] == ARMED:       # the log stopped first
                why.append(f'{start} -> still armed when the log ends')
            else:
                why.append(f"{start} -> {b:.1f}s "
                           f"({ARM_REASON.get(int(st['latest_disarming_reason'][j]), '?')})")
        lines.append('armed: ' + ('; '.join(why) if why else 'never'))
    lines += flight_facts(u)

    modes = mode_changes(u)
    if modes:
        trail, was_armed = [], None
        for t, nav, arm, fs in modes:
            armed_now = arm == 'ARMED'
            mark = '' if armed_now == was_armed else (' armed' if armed_now else ' disarmed')
            trail.append(f"{max(t, 0):.1f} {nav}{mark}{' FAILSAFE' if fs else ''}")
            was_armed = armed_now
        lines.append('modes: ' + ' -> '.join(trail[:12]) + (' ...' if len(trail) > 12 else ''))
        fs = [t for t, _n, _a, f in modes if f]
        lines.append(f'failsafe: first at {fs[0]:.1f}s' if fs else 'failsafe: none')

    lp = topic(u, 'vehicle_local_position')
    if lp is not None and armed:
        t = secs(u, lp['timestamp'])
        during = np.zeros(len(t), dtype=bool)
        for a, b in armed:
            during |= (t >= a) & (t <= b)
        bad = intervals(t, during & ~lp['xy_valid'].astype(bool))
        dr = intervals(t, during & lp['dead_reckoning'].astype(bool))
        if bad or dr:
            lines.append(f'estimator while armed: horizontal position invalid {spans(bad)}; '
                         f'dead reckoning {spans(dr)}')

    notable = dedup(messages(u, 'WARNING'))
    if notable:
        lines.append(f'warnings and errors ({len(notable)} distinct):')
        for t, lvl, text, n, last in notable[:15]:
            lines.append(f'  {t:6.1f}s {lvl:8s} {text}' + (f'  (x{n}, last {last:.1f}s)' if n > 1 else ''))
    lines.append('SITL ground truth: '
                 + ('yes (estimate_vs_truth)' if topic(u, 'vehicle_local_position_groundtruth') is not None
                    else 'not in this log'))
    return '\n'.join(lines)


def timeline(u, path, limit):
    rows = []
    for t, nav, arm, fs in mode_changes(u):
        rows.append((t, f'mode {nav}, {arm}' + (', FAILSAFE' if fs else '')))
    cmd = topic(u, 'vehicle_command')
    if cmd is not None:
        for i, t in enumerate(secs(u, cmd['timestamp'])):
            rows.append((t, f"command {name(VEHICLE_CMD, cmd['command'][i])}"
                            f"(p1={cmd['param1'][i]:g}, p2={cmd['param2'][i]:g})"))
    ack = topic(u, 'vehicle_command_ack')
    if ack is not None:
        for i, t in enumerate(secs(u, ack['timestamp'])):
            rows.append((t, f"ack {name(VEHICLE_CMD, ack['command'][i])}: "
                            f"{name(CMD_RESULT, ack['result'][i])}"))
    ff = topic(u, 'failsafe_flags')
    if ff is not None:
        t = secs(u, ff['timestamp'])
        for k, v in ff.items():
            if k == 'timestamp' or k.startswith('mode_req') or v.dtype.kind not in 'biu':
                continue
            for tt, val in changes(t, v.astype(bool))[1:]:
                rows.append((tt, f"failsafe flag {k} {'SET' if val else 'cleared'}"))
    land = topic(u, 'vehicle_land_detected')
    if land is not None:
        for tt, val in changes(secs(u, land['timestamp']), land['landed'].astype(bool))[1:]:
            rows.append((tt, 'landed' if val else 'takeoff detected'))
    lp = topic(u, 'vehicle_local_position')
    if lp is not None:
        t = secs(u, lp['timestamp'])
        for field in ('xy_valid', 'z_valid', 'dead_reckoning'):
            for tt, val in changes(t, lp[field].astype(bool))[1:]:
                rows.append((tt, f'estimator {field} -> {bool(val)}'))
    for t, lvl, text, n, last in dedup(messages(u, 'WARNING')):
        rows.append((t, f'{lvl}: {text}' + (f'  (x{n} until {last:.1f}s)' if n > 1 else '')))
    rows.sort(key=lambda r: r[0])
    lines = [header(u, path), f'{len(rows)} entries']
    lines += [f'{max(t, 0):7.1f}s  {what}' for t, what in rows[:limit]]
    if len(rows) > limit:
        lines.append(f'... {len(rows) - limit} more (raise limit)')
    return '\n'.join(lines)


# --- estimator ---

def estimator_report(u, path):
    lines = [header(u, path)]
    lp = topic(u, 'vehicle_local_position')
    if lp is not None:
        t = secs(u, lp['timestamp'])
        for field in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid'):
            lines.append(f'{field} false: {spans(intervals(t, ~lp[field].astype(bool)))}')
        lines.append(f"dead reckoning: {spans(intervals(t, lp['dead_reckoning'].astype(bool)))}")
        for field in ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter'):
            resets = [f'{tt:.1f}s' for tt, _ in changes(t, lp[field])[1:]]
            if resets:
                lines.append(f"{field.replace('_counter', '')}s at {', '.join(resets)}")
    flags = topic(u, 'estimator_status_flags')
    if flags is not None:
        t = secs(u, flags['timestamp'])
        on, faults = [], []
        for k, v in flags.items():
            if not (k.startswith('cs_') or k.startswith('fs_')) or v.dtype.kind not in 'biu':
                continue
            for tt, val in changes(t, v.astype(bool))[1:]:
                (faults if k.startswith('fs_') else on).append((tt, f"{k[3:]} {'on' if val else 'off'}"))
        lines.append('fusion/status changes: '
                     + ('; '.join(f'{tt:.1f}s {what}' for tt, what in sorted(on)[:40]) or 'none'))
        lines.append('faults: ' + ('; '.join(f'{tt:.1f}s {what}' for tt, what in sorted(faults)) or 'none'))
    ratios = topic(u, 'estimator_innovation_test_ratios')
    if ratios is not None:
        t = secs(u, ratios['timestamp'])
        over = []
        for k, v in ratios.items():
            v = np.where(np.isfinite(v), v, np.nan)   # inf marks a reset, not a rejection
            if k in ('timestamp', 'timestamp_sample') or not np.isfinite(v).any():
                continue
            i = int(np.nanargmax(v))
            if v[i] > 1.0:
                over.append(f'{k} {v[i]:.1f} at {t[i]:.1f}s')
        lines.append('innovation test ratios above 1 (rejected): ' + ('; '.join(over) or 'none'))
    return '\n'.join(lines)


# --- estimate against SITL truth ---

def truth_report(u, path):
    est, gt = topic(u, 'vehicle_local_position'), topic(u, 'vehicle_local_position_groundtruth')
    if est is None or gt is None:
        return header(u, path) + '\nno vehicle_local_position_groundtruth: only SITL logs carry the truth'
    t = secs(u, est['timestamp'])
    tg = secs(u, gt['timestamp'])
    g = {k: np.interp(t, tg, gt[k]) for k in ('x', 'y', 'z')}
    dn, de = est['x'] - g['x'], est['y'] - g['y']
    dup = -(est['z'] - g['z'])                       # estimated height minus true height
    horiz = np.hypot(dn, de)
    lifted = np.flatnonzero(-g['z'] > 0.3)
    lines = [header(u, path),
             'error = estimate - truth; "height" positive means PX4 thinks it is higher than it is']

    def block(label, m):
        if not m.any():
            return
        lines.append(f'{label}: north {dn[m].mean():+.2f}, east {de[m].mean():+.2f} m '
                     f'(horizontal mean {horiz[m].mean():.2f}, max {horiz[m].max():.2f}); '
                     f'height {dup[m].mean():+.2f} m (range {dup[m].min():+.2f} .. {dup[m].max():+.2f})')

    if len(lifted):
        k = lifted[0]
        block('on the ground, 3 s before liftoff', (t < t[k]) & (t > t[k] - 3))
        block('in the air (truth above 1 m)', -g['z'] > 1.0)
        lines.append(f'liftoff (truth above 0.3 m) at {t[k]:.1f}s')
    else:
        block('whole log (never left the ground)', np.ones(len(t), dtype=bool))
    jump = np.abs(np.diff(np.c_[dn, de, dup], axis=0)).max(axis=1)
    big = np.flatnonzero(jump > 0.3)
    if len(big):
        lines.append('estimate jumped against the truth (>0.3 m in one sample) at '
                     + ', '.join(f'{t[i + 1]:.1f}s ({jump[i]:.2f} m)' for i in big[:8]))
    att, att_gt = topic(u, 'vehicle_attitude'), topic(u, 'vehicle_attitude_groundtruth')
    if att is not None and att_gt is not None:
        ta, tag = secs(u, att['timestamp']), secs(u, att_gt['timestamp'])
        yaw = _yaw(att)
        yaw_gt = np.interp(ta, tag, np.unwrap(_yaw(att_gt)))
        err = np.degrees(np.angle(np.exp(1j * (yaw - yaw_gt))))
        lines.append(f'yaw error: mean {np.abs(err).mean():.1f} deg, max {np.abs(err).max():.1f} deg')
    return '\n'.join(lines)


def _yaw(att):
    w, x, y, z = (att[f'q[{i}]'] for i in range(4))
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# --- velocity tracking ---

def fit_lag(cmd, vel, dt):
    """First-order lag + dead time that best maps commands to measured velocity:
    (tau, dead_time, rmse). cmd/vel are (N, 2) on a uniform dt grid."""
    best = (math.inf, 0.0, 0.0)
    for tau in np.arange(0.05, 2.0001, 0.05):
        keep = math.exp(-dt / tau)
        for k in range(int(round(0.75 / dt)) + 1):
            u = np.vstack([np.repeat(cmd[:1], k, axis=0), cmd])[:len(cmd)]
            sim = np.empty_like(vel)
            sim[0] = vel[0]
            for i in range(1, len(vel)):
                sim[i] = keep * sim[i - 1] + (1 - keep) * u[i - 1]
            err = float(np.sqrt(((sim - vel) ** 2).mean()))
            if err < best[0]:
                best = (err, float(tau), k * dt)
    return best[1], best[2], best[0]


def tracking_report(u, path):
    sp, lp = topic(u, 'trajectory_setpoint'), topic(u, 'vehicle_local_position')
    head = header(u, path)
    if sp is None or lp is None:
        return head + '\nneeds trajectory_setpoint and vehicle_local_position'
    ts = secs(u, sp['timestamp'])
    vel_cmd = np.c_[sp['velocity[0]'], sp['velocity[1]']]
    pos_cmd = np.c_[sp['position[0]'], sp['position[1]']]
    # PX4 offboard flies a velocity setpoint when the position is NaN; a position
    # setpoint with untouched (zero) velocity fields is not a velocity command
    vel_mode = np.isfinite(vel_cmd).all(axis=1) & ~np.isfinite(pos_cmd).any(axis=1)
    if vel_mode.sum() < 10:
        return head + '\nno horizontal velocity setpoints (position NaN, velocity set) in this log'
    # the longest stretch of velocity setpoints with no gap over 0.5 s
    idx = np.flatnonzero(vel_mode)
    cuts = np.flatnonzero(np.diff(ts[idx]) > 0.5) + 1
    seg = max(np.split(idx, cuts), key=lambda s: ts[s[-1]] - ts[s[0]])
    dt = 0.05
    grid = np.arange(ts[seg[0]], ts[seg[-1]], dt)
    if len(grid) < 40:
        return head + '\nless than 2 s of continuous offboard velocity control'
    cmd = np.c_[np.interp(grid, ts[seg], vel_cmd[seg, 0]), np.interp(grid, ts[seg], vel_cmd[seg, 1])]
    tl = secs(u, lp['timestamp'])
    vel = np.c_[np.interp(grid, tl, lp['vx']), np.interp(grid, tl, lp['vy'])]
    tau, dead, rmse = fit_lag(cmd, vel, dt)
    rate = len(seg) / max(ts[seg[-1]] - ts[seg[0]], 1e-3)
    lines = [head,
             f'velocity control {grid[0]:.1f}-{grid[-1]:.1f}s ({len(grid) * dt:.1f} s), '
             f'setpoints logged at {rate:.0f} Hz',
             f'best fit: first-order lag tau {tau:.2f} s after {dead:.2f} s dead time '
             f'(reaction time ~{tau + dead:.2f} s), rmse {rmse:.3f} m/s',
             f'commanded speed peak {np.hypot(*cmd.T).max():.2f} m/s, '
             f'measured peak {np.hypot(*vel.T).max():.2f} m/s',
             'this is PX4 from receiving the setpoint; an offboard planner also sees its own '
             'transport and scheduling delay (fit its log for the end-to-end figure)']
    if rate < 15:
        lines.append(f'the logger thinned trajectory_setpoint to {rate:.0f} Hz; dead time is good to about '
                     f'{1 / rate:.1f} s')
    return '\n'.join(lines)


# --- parameters and raw topics ---

def params_report(u, path, names, pattern):
    params = u.initial_parameters
    picked = {k: v for k, v in params.items()
              if k in names or (pattern and fnmatch.fnmatch(k.upper(), pattern.upper()))}
    lines = [header(u, path), f'{len(params)} parameters in the log, {len(picked)} shown']
    lines += [f'{k} = {v:g}' if isinstance(v, float) else f'{k} = {v}' for k, v in sorted(picked.items())]
    missing = [n for n in names if n not in params]
    if missing:
        lines.append('not in this log: ' + ', '.join(missing))
    changed = [(secs(u, ts).item(), k, v) for ts, k, v in u.changed_parameters]
    if changed:
        lines.append('changed during the log: '
                     + '; '.join(f'{t:.1f}s {k}={v:g}' for t, k, v in changed[:30]))
    return '\n'.join(lines)


def topics_report(u, path):
    lines = [header(u, path), f'{len(u.data_list)} topics (name[instance]: samples, rate, fields)']
    for d in sorted(u.data_list, key=lambda d: (d.name, d.multi_id)):
        t = d.data['timestamp']
        span = (t[-1] - t[0]) / 1e6 if len(t) > 1 else 0
        rate = f'{len(t) / span:.0f} Hz' if span > 0 else '-'
        tag = d.name + (f'[{d.multi_id}]' if d.multi_id else '')
        lines.append(f'{tag}: {len(t)}, {rate}, {len(d.data) - 1} fields')
    return '\n'.join(lines)


def topic_data_report(u, path, topic_name, fields, t_from, t_to, points):
    base, _, inst = topic_name.partition('[')
    data = topic(u, base, int(inst.rstrip(']')) if inst else 0)
    if data is None:
        near = sorted({d.name for d in u.data_list if base.split('_')[0] in d.name})[:10]
        raise ValueError(f"no topic '{topic_name}' in this log" + (f"; similar: {', '.join(near)}" if near else ''))
    t = secs(u, data['timestamp'])
    m = np.ones(len(t), dtype=bool)
    if t_from is not None:
        m &= t >= t_from
    if t_to is not None:
        m &= t <= t_to
    names_all = [k for k in data if k != 'timestamp']
    chosen = [f for f in fields if f in data] if fields else names_all
    unknown = [f for f in fields if f not in data]
    lines = [header(u, path), f'{topic_name}: {m.sum()} of {len(t)} samples'
             + (f' in {t_from}..{t_to} s' if t_from is not None or t_to is not None else '')]
    if unknown:
        lines.append('unknown fields: ' + ', '.join(unknown) + ' (fields: ' + ', '.join(names_all[:60]) + ')')
    if not m.any():
        return '\n'.join(lines)
    for f in chosen:
        v = data[f][m].astype(float)
        if np.isfinite(v).any():
            lines.append(f'{f}: min {np.nanmin(v):.4g}, mean {np.nanmean(v):.4g}, max {np.nanmax(v):.4g}, '
                         f'first {v[0]:.4g}, last {v[-1]:.4g}')
        else:
            lines.append(f'{f}: all NaN')
    if points > 0:
        idx = np.unique(np.linspace(0, m.sum() - 1, min(points, m.sum())).astype(int))
        tt = t[m][idx]
        lines.append('t, ' + ', '.join(chosen))
        for j, i in enumerate(idx):
            lines.append(f'{tt[j]:.2f}, ' + ', '.join(f'{float(data[f][m][i]):.4g}' for f in chosen))
    return '\n'.join(lines)


# --- many logs ---

def scan_line(path, contains):
    """One line about a log, and how many warnings (matching `contains`) it has."""
    u = logs.load(path, ('vehicle_status', 'event'))
    armed = armed_intervals(u)
    fs = [t for t, _n, _a, f in mode_changes(u) if f]
    msgs = messages(u, 'WARNING', contains)
    first = f'{msgs[0][0]:.1f}s {msgs[0][3]}' if msgs else ''
    line = (f"{os.path.basename(path)}  {logs.local(logs.start_time(path))[11:19]}  "
            f"{duration(u):5.1f}s  armed {sum(b - a for a, b in armed):5.1f}s  "
            f"{'FAILSAFE ' + format(fs[0], '.1f') + 's' if fs else 'no failsafe':16s} "
            f"{len(msgs):3d} warnings  {first[:90]}")
    return line, len(msgs)


# --- plot ---

def plot_png(u, path, series, t_from, t_to):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(len(series), 1, figsize=(9, 2.2 * len(series)), sharex=True, squeeze=False)
    for ax, s in zip(axes[:, 0], series):
        for one in s.split('+'):
            tname, _, field = one.strip().rpartition('.')
            base, _, inst = tname.partition('[')
            data = topic(u, base, int(inst.rstrip(']')) if inst else 0)
            if data is None or field not in data:
                raise ValueError(f"no '{one.strip()}' in this log (use topic.field, e.g. vehicle_local_position.z)")
            t = secs(u, data['timestamp'])
            m = np.ones(len(t), dtype=bool)
            if t_from is not None:
                m &= t >= t_from
            if t_to is not None:
                m &= t <= t_to
            ax.plot(t[m], data[field][m], lw=1.2, label=one.strip())
        for t_mode, nav, _arm, fs in mode_changes(u):
            ax.axvline(t_mode, color='#d03b3b' if fs else '#b5b4ad', lw=0.8, ls='--')
        ax.legend(fontsize=7, loc='best')
        ax.grid(alpha=0.3)
    axes[-1, 0].set_xlabel('s since log start (dashed: mode change, red: failsafe)')
    fig.suptitle(os.path.basename(path), fontsize=9)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=90)
    plt.close(fig)
    return buf.getvalue()
