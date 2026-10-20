"""Cut a real ULog down to a small test fixture.

  uv run python tools/make_fixture.py in.ulg out.ulg

Keeps the topics the tools read. Event-like topics (status, commands, flags)
are kept whole; the fast continuous ones are thinned (position 20 Hz, which the
velocity-lag fit needs; attitude and innovations 10 Hz).
"""

import sys

import numpy as np
from pyulog import ULog

KEEP = [
    'vehicle_status', 'vehicle_command', 'vehicle_command_ack', 'failsafe_flags', 'event',
    'vehicle_land_detected', 'estimator_status_flags', 'trajectory_setpoint', 'offboard_control_mode',
    'vehicle_local_position', 'vehicle_local_position_groundtruth', 'vehicle_attitude',
    'vehicle_attitude_groundtruth', 'estimator_innovation_test_ratios',
]
THIN = {'vehicle_local_position': 20, 'vehicle_local_position_groundtruth': 20, 'vehicle_attitude': 10,
        'vehicle_attitude_groundtruth': 10, 'estimator_innovation_test_ratios': 10}


def main(src, dst):
    u = ULog(src, KEEP)
    for d in u.data_list:
        if d.name in THIN:
            t = d.data['timestamp']
            keep = np.unique(np.searchsorted(t, np.arange(t[0], t[-1], 1e6 / THIN[d.name])))
            d.data = {k: v[keep] for k, v in d.data.items()}
    u.write_ulog(dst)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
