"""Finding logs, naming their start time, and keeping a few parsed ones around."""

import functools
import glob
import json
import os
import re
from datetime import datetime, timezone

from pyulog import ULog
from pyulog.px4_events import PX4Events

DEFAULT_DIRS = ['~/PX4-Autopilot/build/px4_sitl_default/rootfs/log']
PX4_NAME = re.compile(r'(\d{4}-\d{2}-\d{2})[/\\](\d{2})_(\d{2})_(\d{2})\.ulg$')


def log_dirs():
    """PX4_ULOG_DIRS (os.pathsep separated) or the SITL default."""
    raw = os.environ.get('PX4_ULOG_DIRS')
    dirs = raw.split(os.pathsep) if raw else DEFAULT_DIRS
    return [os.path.expanduser(d) for d in dirs if d]


def all_logs():
    """Every .ulg under the log dirs, oldest first."""
    found = set()
    for d in log_dirs():
        found.update(glob.glob(os.path.join(d, '**', '*.ulg'), recursive=True))
    return sorted(found, key=lambda p: (start_time(p), p))


def start_time(path):
    """When the log started, as an aware datetime. PX4 names logs
    <UTC date>/<UTC hh_mm_ss>.ulg once it knows the time; otherwise fall back
    to the file's modification time (end of the log, close enough to sort)."""
    m = PX4_NAME.search(path)
    if m:
        day, hh, mm, ss = m.groups()
        return datetime.strptime(f'{day} {hh}:{mm}:{ss}', '%Y-%m-%d %H:%M:%S').replace(
            tzinfo=timezone.utc)
    return datetime.fromtimestamp(os.path.getmtime(path), tz=timezone.utc)


def local(dt):
    return dt.astimezone().strftime('%Y-%m-%d %H:%M:%S %Z')


def resolve(ref):
    """A log from a path, a file name ('16_21_32.ulg' or '2026-10-20/16_21_32'),
    'latest', or 'latest-N' (N logs before the latest)."""
    ref = ref.strip()
    m = re.fullmatch(r'latest(?:-(\d+))?', ref)
    if m:
        logs = all_logs()
        k = int(m.group(1) or 0)
        if k >= len(logs):
            raise ValueError(f'only {len(logs)} logs under {log_dirs()}')
        return logs[-1 - k]
    path = os.path.expanduser(ref)
    if os.path.isfile(path):
        return os.path.abspath(path)
    want = ref if ref.endswith('.ulg') else ref + '.ulg'
    hits = [p for p in all_logs() if p.endswith(os.sep + want) or p.endswith('/' + want)]
    if not hits:
        raise ValueError(f"no log matches '{ref}' under {log_dirs()}")
    if len(hits) > 1:
        raise ValueError(f"'{ref}' matches {len(hits)} logs, give the date too: "
                         + ', '.join(os.path.relpath(h, os.path.dirname(os.path.dirname(h)))
                                     for h in hits[:5]))
    return hits[0]


def find_at(when):
    """The log that was running at local time `when` ('HH:MM[:SS]' today or
    'YYYY-MM-DD HH:MM[:SS]')."""
    text = when.strip()
    if '-' not in text:
        text = datetime.now().strftime('%Y-%m-%d ') + text
    if text.count(':') == 1:
        text += ':00'
    # a naive datetime's astimezone() reads it as local time, with that date's DST
    t = datetime.strptime(text, '%Y-%m-%d %H:%M:%S').astimezone(timezone.utc)
    for path in reversed(all_logs()):
        start = start_time(path)
        if start <= t:
            u = load(path, ('vehicle_status',))
            end = start.timestamp() + (u.last_timestamp - u.start_timestamp) / 1e6
            return path if t.timestamp() <= end + 5 else None
    return None


def load(path, topics=None):
    """Parsed log; topics=None reads every topic. Cached on the file's mtime."""
    return _load(path, os.path.getmtime(path), tuple(sorted(topics)) if topics else None)


@functools.lru_cache(maxsize=12)
def _load(path, _mtime, topics):
    return ULog(path, list(topics) if topics else None, disable_str_exceptions=True)


def events(ulog):
    """Decoded PX4 events as (timestamp, level, text). Uses the definitions
    embedded in the log, else a local all_events.json from a PX4 build; never
    downloads."""
    parser = PX4Events()
    parser.set_default_json_definitions_cb(_local_event_definitions)
    try:
        return parser.get_logged_events(ulog)
    except Exception:          # undecodable metadata: report the log without events
        return []


def _local_event_definitions(already_loaded):
    if already_loaded:
        return None
    for d in log_dirs():
        build = os.path.normpath(os.path.join(d, '..', '..'))
        path = os.path.join(build, 'events', 'all_events.json')
        if os.path.isfile(path):
            with open(path) as f:
                return json.load(f)
    return None
