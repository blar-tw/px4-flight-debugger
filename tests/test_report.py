"""The reports on two real SITL flights (trimmed, see tools/make_fixture.py).
Each test is a question that once took a hand-written pyulog script."""

import re
from datetime import timedelta

import numpy as np
import pytest

import conftest
from px4_ulog_mcp import logs, report, server
from px4_ulog_mcp.names import ARM_REASON
from conftest import COMPASS, DRIFT


def number(pattern, text):
    return float(re.search(pattern, text).group(1))


# --- finding logs ---

def test_logs_are_found_and_named_by_their_utc_start():
    paths = logs.all_logs()
    assert [p[-12:] for p in paths] == ['16_21_32.ulg', '17_06_45.ulg']
    assert logs.start_time(paths[0]).isoformat() == '2026-10-20T16:21:32+00:00'


@pytest.mark.parametrize('ref', ['latest', '17_06_45', '17_06_45.ulg', DRIFT])
def test_resolve_accepts_the_documented_forms(ref):
    assert logs.resolve(ref).endswith('17_06_45.ulg')


def test_resolve_latest_minus_one_and_unknown():
    assert logs.resolve('latest-1').endswith('16_21_32.ulg')
    with pytest.raises(ValueError, match='no log matches'):
        logs.resolve('12_00_00')


def test_find_at_maps_local_wall_clock_to_the_running_log():
    start = logs.start_time(logs.resolve(DRIFT)).astimezone()
    for later in (0, 30):
        when = (start + timedelta(seconds=later)).strftime('%Y-%m-%d %H:%M:%S')
        assert logs.find_at(when).endswith('17_06_45.ulg')
    assert logs.find_at((start + timedelta(minutes=5)).strftime('%Y-%m-%d %H:%M')) is None


# --- what happened ---

def test_summary_tells_the_compass_story():
    out = server.log_summary(COMPASS)
    assert 'v1.14.4' in out
    assert 'armed: 12.6s (internal command) -> 49.9s (landing)' in out
    assert '17.6 ALTCTL FAILSAFE' in out
    assert 'No valid data from Compass 0' in out
    assert 'horizontal position invalid 17.5-50.0s' in out


def test_summary_of_a_clean_flight_says_so():
    out = server.log_summary(DRIFT)
    assert 'failsafe: none' in out
    assert 'still armed when the log ends' in out


def test_timeline_puts_the_cause_before_the_failsafe_and_collapses_repeats():
    out = server.flight_timeline(COMPASS)
    lines = out.splitlines()
    first = lambda text: next(i for i, ln in enumerate(lines) if text in ln)
    assert first('Compass 0') < first('xy_valid -> False') < first('ALTCTL, ARMED, FAILSAFE')
    assert first('ack COMPONENT_ARM_DISARM: accepted') < first('takeoff detected')
    assert 'Failsafe: blind land  (x15 until 46.1s)' in out
    assert out.count('blind land') == 1


def test_estimator_health_shows_the_magnetometer_dropping_out():
    out = server.estimator_health(COMPASS)
    assert '13.5s mag_hdg off' in out
    assert '22.5s gps off' in out
    assert 'dead reckoning: 0.0-4.9s, 13.5-50.9s' in out


def test_messages_filter_by_text_and_level():
    out = server.log_messages(COMPASS, min_level='INFO', contains='compass')
    assert '3 messages' in out
    assert '1 messages at ERROR or worse' in server.log_messages(COMPASS, min_level='ERROR')


# --- numbers ---

def test_estimate_vs_truth_reports_the_height_offset_on_the_ground_and_in_the_air():
    out = server.estimate_vs_truth(DRIFT)
    ground = number(r'before liftoff:.*height ([+-][\d.]+)', out)
    air = number(r'in the air.*height ([+-][\d.]+)', out)
    assert 0.2 < ground < 0.45 and 0.4 < air < 0.6     # PX4 thinks it is higher than it is
    assert 'liftoff (truth above 0.3 m) at 19.8s' in out


def test_velocity_tracking_fits_px4s_lag():
    out = server.velocity_tracking(DRIFT)
    tau = number(r'tau ([\d.]+)', out)
    dead = number(r'after ([\d.]+) s dead', out)
    assert 0.35 <= tau <= 0.7 and 0.05 <= dead <= 0.3
    assert number(r'rmse ([\d.]+)', out) < 0.05


def test_lag_fit_recovers_a_known_system():
    dt, tau, dead = 0.05, 0.6, 0.2
    rng = np.random.default_rng(0)
    cmd = np.repeat(rng.uniform(-1, 1, (40, 2)), 20, axis=0)       # 1 s steps
    keep, k = np.exp(-dt / tau), round(dead / dt)
    vel = np.zeros_like(cmd)
    for i in range(1, len(cmd)):
        vel[i] = keep * vel[i - 1] + (1 - keep) * cmd[max(i - 1 - k, 0)]
    fit_tau, fit_dead, rmse = report.fit_lag(cmd, vel, dt)
    assert fit_tau == pytest.approx(tau) and fit_dead == pytest.approx(dead) and rmse < 1e-6


def test_params_and_topic_data():
    out = server.log_params(COMPASS, names=['NAV_DLL_ACT', 'NO_SUCH'], pattern='EKF2_MAG_T*')
    assert 'NAV_DLL_ACT = 0' in out and 'EKF2_MAG_TYPE = ' in out and 'not in this log: NO_SUCH' in out
    out = server.topic_data('vehicle_local_position', DRIFT, fields=['z'], t_from=30, t_to=35, points=3)
    assert -2.3 < number(r'z: min ([-\d.]+)', out) < -1.9
    assert len(out.splitlines()) == 3 + 1 + 1 + 3       # header, count, stats, table head, rows
    with pytest.raises(Exception, match='no topic'):
        server.topic_data('vehicle_local_positon', DRIFT)


def test_scan_finds_the_compass_failure_among_logs():
    out = server.scan_logs(contains='compass')
    assert '1 with warnings' in out
    assert '16_21_32.ulg' in out and '17_06_45.ulg' not in out


def test_plot_returns_a_png():
    img = server.plot_series(['vehicle_local_position.z+vehicle_local_position_groundtruth.z'], DRIFT)
    assert img.data[:8] == b'\x89PNG\r\n\x1a\n'
    with pytest.raises(Exception, match="no 'vehicle_local_position.nope'"):
        server.plot_series(['vehicle_local_position.nope'], DRIFT)


# --- names ---

def test_px4_version_and_reason_names():
    assert report.px4_version({'ver_sw_release': 0x010E0400}) == 'v1.14.4 dev'
    assert report.px4_version({'ver_sw_release': 0x010F00FF}) == 'v1.15.0'
    assert ARM_REASON[6] == 'landing'        # not the stale 'safety button' from VehicleStatus.msg


# --- found in review: robustness ---

def fresh(ref):
    """A private parse of a fixture, safe to modify."""
    from pyulog import ULog
    path = logs.resolve(ref)
    return path, ULog(path)


def test_logs_without_dead_reckoning_still_report():
    # GIVEN a log like PX4 v1.13's, whose vehicle_local_position has no dead_reckoning
    path, u = fresh(COMPASS)
    del report.topic(u, 'vehicle_local_position')['dead_reckoning']
    # THEN every report that reads it still works
    assert 'horizontal position invalid 17.5-50.0s' in report.summary(u, path)
    assert 'dead_reckoning' not in report.timeline(u, path, 200)
    assert 'dead reckoning' not in report.estimator_report(u, path)


def test_unreadable_files_are_errors_and_do_not_stop_a_scan(tmp_path, monkeypatch):
    day = tmp_path / '2026-10-21'
    day.mkdir()
    good = open(logs.resolve(DRIFT), 'rb').read()
    (day / '10_00_00.ulg').write_bytes(good[:3000])       # cut off inside the definitions
    (day / '10_05_00.ulg').write_bytes(b'')
    monkeypatch.setenv('PX4_ULOG_DIRS', f'{tmp_path}:{conftest.FIXTURES}')
    with pytest.raises(Exception, match='not a readable ULog'):
        server.log_summary('10_05_00')
    out = server.scan_logs(limit=10)
    assert '10_05_00.ulg  skipped' in out          # empty: rejected, with the reason
    assert '10_00_00.ulg  2026' in out             # truncated: pyulog reads what is there
    assert '17_06_45.ulg' in out


def test_reason_names_cover_failsafe_and_show_unknown_numbers():
    from px4_ulog_mcp.names import name
    assert name(ARM_REASON, 14) == 'failsafe' and name(ARM_REASON, 99) == '99'


def test_height_and_distance_count_from_arming():
    out = server.log_summary(DRIFT)
    assert 1.8 < number(r'up to ([\d.]+) m higher', out) < 2.3
    assert 13 < number(r'([\d.]+) m away', out) < 15


def test_truth_is_compared_from_where_the_drone_spawned():
    path, u = fresh(DRIFT)
    before = report.truth_report(u, path)
    gt = report.topic(u, 'vehicle_local_position_groundtruth')
    gt['x'] = gt['x'] + 5.0                                  # spawned 5 m north, 3 m east
    gt['y'] = gt['y'] + 3.0
    after = report.truth_report(u, path)
    assert 'spawned at world north +5.00, east +3.00 m' in after
    pick = lambda text: re.search(r'in the air.*', text).group(0)
    assert pick(after) == pick(before)
    assert 'local origin +0.27 m north' in before or 'local origin -0.27 m north' in before


def test_velocity_tracking_flags_a_bad_fit_and_a_flight_without_velocity_control():
    path, u = fresh(DRIFT)
    lp = report.topic(u, 'vehicle_local_position')
    lp['vx'] = np.random.default_rng(1).normal(0, 1, len(lp['vx'])).astype(lp['vx'].dtype)
    assert 'POOR FIT' in report.tracking_report(u, path)
    assert 'no offboard horizontal velocity setpoints' in server.velocity_tracking(COMPASS)


def test_bad_inputs_say_what_is_wrong():
    with pytest.raises(Exception, match='not one of'):
        server.log_messages(COMPASS, min_level='warn')
    with pytest.raises(Exception, match='not a topic name'):
        server.topic_data('sensor_mag[x]', COMPASS)
    assert logs.find_at('2026-10-20T10:07:00') is None or logs.find_at('2026-10-20T10:07:00').endswith('.ulg')


def test_dedup_keeps_different_numbers_apart():
    msgs = [(1.0, 'WARNING', 'console', 'Compass 0 bad'), (2.0, 'WARNING', 'console', 'Compass 1 bad'),
            (3.0, 'WARNING', 'console', 'Compass 0 bad')]
    assert [(text, n) for _t, _l, text, n, _last in report.dedup(msgs)] == [('Compass 0 bad', 2), ('Compass 1 bad', 1)]
