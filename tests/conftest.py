import os

import pytest

FIXTURES = os.path.join(os.path.dirname(__file__), 'fixtures')
COMPASS = '2026-10-20/16_21_32'   # SITL lost its compass after arming, failsafe, blind landing
DRIFT = '2026-10-20/17_06_45'     # clean offboard velocity flight, height estimate drifting


@pytest.fixture(autouse=True)
def fixture_logs(monkeypatch):
    monkeypatch.setenv('PX4_ULOG_DIRS', FIXTURES)


@pytest.fixture
def anyio_backend():
    return 'asyncio'
