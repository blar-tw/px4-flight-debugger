"""The server over MCP: in-process and as a real stdio subprocess."""

import os
import sys

import pytest
from mcp import Client, StdioServerParameters

from px4_flight_debugger.server import server
from conftest import COMPASS, FIXTURES

TOOLS = {'list_logs', 'log_summary', 'log_messages', 'flight_timeline', 'estimator_health',
         'estimate_vs_truth', 'velocity_tracking', 'log_params', 'list_topics', 'topic_data',
         'scan_logs', 'plot_series'}


@pytest.mark.anyio
async def test_in_process_tools_and_a_call():
    async with Client(server) as client:
        tools = {t.name: t for t in (await client.list_tools()).tools}
        assert set(tools) == TOOLS
        schema = tools['topic_data'].input_schema
        assert schema['required'] == ['topic'] and 'points' in schema['properties']
        result = await client.call_tool('log_summary', {'log': COMPASS})
        assert not result.is_error
        assert 'No valid data from Compass 0' in result.content[0].text


@pytest.mark.anyio
async def test_bad_log_is_a_tool_error_not_a_crash():
    async with Client(server) as client:
        result = await client.call_tool('log_summary', {'log': 'nope'})
        assert result.is_error
        assert 'no log matches' in result.content[0].text


@pytest.mark.anyio
async def test_stdio_subprocess():
    params = StdioServerParameters(command=sys.executable, args=['-m', 'px4_flight_debugger'],
                                   env={**os.environ, 'PX4_ULOG_DIRS': FIXTURES})
    async with Client(params) as client:
        result = await client.call_tool('scan_logs', {'contains': 'compass'})
        assert '16_21_32.ulg' in result.content[0].text
        image = await client.call_tool('plot_series', {'series': ['vehicle_status.nav_state'], 'log': COMPASS})
        assert image.content[0].type == 'image' and image.content[0].mime_type == 'image/png'


@pytest.mark.anyio
async def test_argument_limits_are_enforced():
    async with Client(server) as client:
        result = await client.call_tool('scan_logs', {'limit': 0})
        assert result.is_error


@pytest.mark.anyio
async def test_a_truncated_log_over_stdio_is_an_error_and_the_channel_survives(tmp_path):
    # pyulog prints warnings on stdout; on stdio that is the protocol channel
    day = tmp_path / '2026-10-21'
    day.mkdir()
    data = open(os.path.join(FIXTURES, '2026-10-20', '17_06_45.ulg'), 'rb').read()
    (day / '10_00_00.ulg').write_bytes(data[:len(data) // 2])   # cut off mid-data
    (day / '10_05_00.ulg').write_bytes(data[:3000])              # cut off in the definitions
    (day / '10_10_00.ulg').write_bytes(b'')
    params = StdioServerParameters(command=sys.executable, args=['-m', 'px4_flight_debugger'],
                                   env={**os.environ, 'PX4_ULOG_DIRS': str(tmp_path)})
    async with Client(params) as client:
        half = await client.call_tool('log_summary', {'log': '10_00_00'})
        assert not half.is_error and 'PX4 v1.14.4' in half.content[0].text
        # pyulog reads this one but warns "File corruption detected" on stdout
        cut = await client.call_tool('log_summary', {'log': '10_05_00'})
        assert not cut.is_error and '10_05_00.ulg' in cut.content[0].text
        empty = await client.call_tool('log_summary', {'log': '10_10_00'})
        assert empty.is_error and 'not a readable ULog' in empty.content[0].text
        again = await client.call_tool('list_logs', {})
        assert '3 logs' in again.content[0].text
