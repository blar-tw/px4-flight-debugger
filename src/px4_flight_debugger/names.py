"""Readable names for PX4 enums, taken from the v1.14 message definitions.
Values a newer or older PX4 adds show up as their number."""

NAV_STATE = {
    0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER', 5: 'AUTO_RTL',
    10: 'ACRO', 12: 'DESCEND', 13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB',
    17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND', 19: 'AUTO_FOLLOW_TARGET', 20: 'AUTO_PRECLAND',
    21: 'ORBIT', 22: 'AUTO_VTOL_TAKEOFF',
}

ARMING_STATE = {0: 'INIT', 1: 'STANDBY', 2: 'ARMED', 3: 'STANDBY_ERROR', 4: 'SHUTDOWN',
                5: 'IN_AIR_RESTORE'}
ARMED = 2

# commander's arm_disarm_reason_t (src/lib/events/enums.json). The ARM_DISARM_REASON_*
# constants in VehicleStatus.msg are out of date and do not match what is logged.
ARM_REASON = {
    0: 'transition to standby', 1: 'RC', 2: 'RC switch', 3: 'internal command',
    4: 'external command', 5: 'mission start', 6: 'landing', 7: 'auto preflight disarming',
    8: 'kill switch', 9: 'lockdown', 10: 'failure detector', 11: 'shutdown', 12: 'unit test',
    13: 'RC button', 14: 'failsafe',
}

VEHICLE_CMD = {
    17: 'NAV_LOITER_UNLIM', 20: 'NAV_RETURN_TO_LAUNCH', 21: 'NAV_LAND', 22: 'NAV_TAKEOFF',
    176: 'DO_SET_MODE', 179: 'DO_SET_HOME', 185: 'DO_FLIGHTTERMINATION', 192: 'DO_REPOSITION',
    209: 'DO_MOTOR_TEST', 241: 'PREFLIGHT_CALIBRATION', 300: 'MISSION_START',
    400: 'COMPONENT_ARM_DISARM', 1000: 'DO_GIMBAL_MANAGER_PITCHYAW',
    1001: 'DO_GIMBAL_MANAGER_CONFIGURE', 100000: 'SET_GPS_GLOBAL_ORIGIN',
}

CMD_RESULT = {0: 'accepted', 1: 'temporarily rejected', 2: 'denied', 3: 'unsupported',
              4: 'failed', 5: 'in progress', 6: 'cancelled'}

# PX4 log levels as pyulog reports them, most severe first
LEVELS = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO', 'DEBUG']


def name(table, value):
    value = int(value)
    return table.get(value, str(value))


def severity(level):
    """Index in LEVELS (lower = worse); unknown levels sort as DEBUG."""
    level = level.upper()
    return LEVELS.index(level) if level in LEVELS else len(LEVELS)
