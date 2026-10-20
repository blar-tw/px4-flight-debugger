"""The server over MCP: in-process and as a real stdio subprocess."""

import os
import sys

import pytest
from mcp import Client, StdioServerParameters

from px4_ulog_mcp.server import server
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
    params = StdioServerParameters(command=sys.executable, args=['-m', 'px4_ulog_mcp'],
                                   env={**os.environ, 'PX4_ULOG_DIRS': FIXTURES})
    async with Client(params) as client:
        result = await client.call_tool('scan_logs', {'contains': 'compass'})
        assert '16_21_32.ulg' in result.content[0].text
        image = await client.call_tool('plot_series', {'series': ['vehicle_status.nav_state'], 'log': COMPASS})
        assert image.content[0].type == 'image' and image.content[0].mime_type == 'image/png'
