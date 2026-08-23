#!/usr/bin/env python3

"""Compare Phoenix mission timing and battery use from DataFlash logs."""

# AP_FLAKE8_CLEAN

import argparse

import numpy as np
from pymavlink import mavutil


def load_log(path):
    wanted = ['ARM', 'ARSP', 'BAT', 'CTUN', 'GPS', 'POS', 'QTUN', 'XKF2']
    channels = {}
    arm_events = []
    connection = mavutil.mavlink_connection(path)
    while True:
        msg = connection.recv_match(type=wanted)
        if msg is None:
            break
        data = msg.to_dict()
        msg_type = msg.get_type()
        time_s = data['TimeUS'] * 1.0e-6
        if msg_type == 'ARM':
            arm_events.append((time_s, data['ArmState']))
        elif msg_type == 'BAT' and data.get('Inst', 0) == 0:
            values = (data['Curr'], data['Volt'], data['CurrTot'], data['EnrgTot'], data['RemPct'])
            channels.setdefault('battery', []).append((time_s, *values))
        elif msg_type == 'ARSP' and data.get('I', 0) == 0:
            channels.setdefault('airspeed', []).append((time_s, data['Airspeed']))
        elif msg_type == 'CTUN':
            channels.setdefault('throttle', []).append((time_s, data['ThO']))
        elif msg_type == 'GPS' and data.get('I', 0) == 0:
            channels.setdefault('groundspeed', []).append((time_s, data['Spd']))
        elif msg_type == 'POS':
            channels.setdefault('altitude', []).append((time_s, data['RelHomeAlt']))
        elif msg_type == 'QTUN':
            channels.setdefault('qthrottle', []).append((time_s, data['ThO']))
        elif msg_type == 'XKF2' and data.get('C', 0) == 0:
            channels.setdefault('wind_north', []).append((time_s, data['VWN']))
            channels.setdefault('wind_east', []).append((time_s, data['VWE']))
    return channels, arm_events


def interpolate(channels, name, times):
    values = np.asarray(channels.get(name, []))
    if len(values) == 0:
        return np.zeros_like(times)
    return np.interp(times, values[:, 0], values[:, 1])


def smooth(values, count):
    return np.convolve(values, np.ones(count) / count, mode='same')


def arm_window(battery, arm_events):
    starts = [time_s for time_s, state in arm_events if state == 1]
    start = starts[0] if starts else battery[0, 0]
    ends = [time_s for time_s, state in arm_events if state == 0 and time_s > start]
    end = ends[0] if ends else battery[-1, 0]
    return start, end


def report(path):
    channels, arm_events = load_log(path)
    battery = np.asarray(channels['battery'])
    start, end = arm_window(battery, arm_events)
    battery = battery[(battery[:, 0] >= start) & (battery[:, 0] <= end)]
    time_s = battery[:, 0]
    time_step = np.diff(time_s, prepend=time_s[0])
    sample_period = np.median(np.diff(time_s))

    airspeed = interpolate(channels, 'airspeed', time_s)
    groundspeed = interpolate(channels, 'groundspeed', time_s)
    throttle = interpolate(channels, 'throttle', time_s)
    qthrottle = interpolate(channels, 'qthrottle', time_s)
    altitude = interpolate(channels, 'altitude', time_s)
    wind_north = interpolate(channels, 'wind_north', time_s)
    wind_east = interpolate(channels, 'wind_east', time_s)
    climb_rate = smooth(np.gradient(altitude, time_s), max(3, round(20 / sample_period)))
    energy_step = np.maximum(0, np.diff(battery[:, 4], prepend=battery[0, 4]))

    fixed_wing = (airspeed > 17) & (qthrottle < 0.1)
    phases = {
        'VTOL/transition': ~fixed_wing,
        'fixed climb': fixed_wing & (climb_rate > 0.5),
        'fixed level': fixed_wing & (np.abs(climb_rate) <= 0.5),
        'fixed descent': fixed_wing & (climb_rate < -0.5),
    }

    mean_wind_north = np.mean(wind_north)
    mean_wind_east = np.mean(wind_east)
    wind_speed = np.hypot(mean_wind_north, mean_wind_east)
    wind_from = (np.degrees(np.arctan2(mean_wind_east, mean_wind_north)) + 180) % 360
    wind_magnitude = np.hypot(wind_north, wind_east)

    print(path)
    print('  arm window: %.2f min, %.3f Ah, %.1f Wh, RemPct %u -> %u' % (
        (end - start) / 60, (battery[-1, 3] - battery[0, 3]) / 1000,
        battery[-1, 4] - battery[0, 4], battery[0, 5], battery[-1, 5]))
    print('  cumulative energy at start/end: %.1f -> %.1f Wh' % (battery[0, 4], battery[-1, 4]))
    print('  wind: mean vector %.2f m/s from %.0f deg; magnitude p50 %.2f, p95 %.2f' % (
        wind_speed, wind_from, np.percentile(wind_magnitude, 50), np.percentile(wind_magnitude, 95)))
    for name, mask in phases.items():
        if not np.any(mask):
            continue
        weights = time_step[mask]
        print('  %-15s %6.2f min %7.1f Wh  I %5.2f A  thr %5.1f%%  EAS %5.2f  GS %5.2f  Vz %+5.2f' % (
            name, np.sum(weights) / 60, np.sum(energy_step[mask]),
            np.average(battery[mask, 1], weights=weights), np.average(throttle[mask], weights=weights),
            np.average(airspeed[mask], weights=weights), np.average(groundspeed[mask], weights=weights),
            np.average(climb_rate[mask], weights=weights)))
    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+')
    args = parser.parse_args()
    for path in args.logs:
        report(path)


if __name__ == '__main__':
    main()
