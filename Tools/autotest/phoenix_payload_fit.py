#!/usr/bin/env python3

"""Fit Phoenix payload mass, drag and electrical load from flight logs."""

# AP_FLAKE8_CLEAN

import argparse

import numpy as np
from pymavlink import mavutil


RHO0 = 1.225
GRAVITY = 9.80665
WING_AREA = 0.45
WING_SPAN = 1.88
OSWALD = 0.9


def add_channel(channels, name, time_s, value):
    channels.setdefault(name, ([], []))
    channels[name][0].append(time_s)
    channels[name][1].append(value)


def load_log(path):
    connection = mavutil.mavlink_connection(path)
    channels = {}
    while True:
        msg = connection.recv_match(type=['ARM', 'ATT', 'ARSP', 'BAT', 'CTUN', 'ESC', 'POS', 'QTUN'])
        if msg is None:
            break
        data = msg.to_dict()
        time_s = data.get('TimeUS', 0) * 1.0e-6
        msg_type = msg.get_type()
        if msg_type == 'ARM':
            add_channel(channels, 'armed', time_s, data['ArmState'])
        elif msg_type == 'ATT':
            add_channel(channels, 'roll', time_s, data['Roll'])
        elif msg_type == 'ARSP' and data.get('I', 0) == 0:
            add_channel(channels, 'airspeed', time_s, data['Airspeed'])
        elif msg_type == 'BAT' and data.get('Inst', data.get('I', 0)) == 0:
            add_channel(channels, 'current', time_s, data['Curr'])
            add_channel(channels, 'voltage', time_s, data['Volt'])
        elif msg_type == 'CTUN':
            add_channel(channels, 'throttle', time_s, data['ThO'])
        elif msg_type == 'ESC':
            add_channel(channels, 'rpm_%u' % data['Instance'], time_s, data['RPM'])
        elif msg_type == 'POS':
            add_channel(channels, 'altitude', time_s, data['RelHomeAlt'])
        elif msg_type == 'QTUN':
            add_channel(channels, 'qthrottle', time_s, data['ThO'])
    return channels


def smooth(values, count):
    if count < 2:
        return values
    kernel = np.ones(count) / count
    return np.convolve(values, kernel, mode='same')


def resample(channels, step=0.2):
    core_names = ('airspeed', 'altitude', 'current', 'roll', 'throttle', 'voltage')
    start = max(min(channels[name][0]) for name in core_names)
    end = min(max(channels[name][0]) for name in core_names)
    time_s = np.arange(start, end, step)
    result = {'time': time_s}
    for name in core_names:
        result[name] = np.interp(time_s, channels[name][0], channels[name][1])
    if 'armed' in channels:
        indices = np.searchsorted(channels['armed'][0], time_s, side='right') - 1
        indices = np.clip(indices, 0, len(channels['armed'][1]) - 1)
        result['armed'] = np.asarray(channels['armed'][1])[indices] > 0
    else:
        result['armed'] = np.ones_like(time_s, dtype=bool)
    if 'qthrottle' in channels:
        result['qthrottle'] = np.interp(time_s, channels['qthrottle'][0], channels['qthrottle'][1],
                                        left=channels['qthrottle'][1][0], right=0.0)
    else:
        result['qthrottle'] = np.zeros_like(time_s)
    for name in channels:
        if name.startswith('rpm_'):
            result[name] = np.interp(time_s, channels[name][0], channels[name][1])
    window = max(3, round(20 / step))
    result['climb_rate'] = smooth(np.gradient(result['altitude'], time_s), window)
    result['acceleration'] = smooth(np.gradient(result['airspeed'], time_s), window)
    return result


def fixed_wing_mask(data):
    return (data['armed'] & (data['airspeed'] > 17) & (data['altitude'] > 80) &
            (data['qthrottle'] < 0.1))


def forward_rpm(data):
    mask = fixed_wing_mask(data)
    rpm_names = [name for name in data if name.startswith('rpm_')]
    ranked = sorted(rpm_names, key=lambda name: np.median(data[name][mask]), reverse=True)
    if len(ranked) < 2:
        raise RuntimeError('fewer than two ESC RPM streams found')
    selected = ranked[:2]
    return 0.5 * (data[selected[0]] + data[selected[1]]), selected


def induced_factor():
    aspect_ratio = WING_SPAN ** 2 / WING_AREA
    return 1.0 / (np.pi * OSWALD * aspect_ratio)


def drag_components(mass, airspeed, roll, parasitic_cd):
    dynamic_area = 0.5 * RHO0 * airspeed ** 2 * WING_AREA
    load_factor = 1.0 / np.cos(np.radians(roll))
    weight = mass * GRAVITY
    induced_drag = induced_factor() * (weight * load_factor) ** 2 / dynamic_area
    return dynamic_area * parasitic_cd, induced_drag


def baseline_parasitic_cd(data, mass):
    mask = (fixed_wing_mask(data) & (data['throttle'] < 3) & (data['climb_rate'] < -0.5) &
            (np.abs(data['roll']) < 12))
    if np.count_nonzero(mask) < 20:
        raise RuntimeError('baseline log has too few steady idle-glide samples')
    airspeed = data['airspeed'][mask]
    descent_rate = -data['climb_rate'][mask]
    dynamic_area = 0.5 * RHO0 * airspeed ** 2 * WING_AREA
    load_factor = 1.0 / np.cos(np.radians(data['roll'][mask]))
    weight = mass * GRAVITY
    drag = weight * descent_rate / airspeed
    induced_drag = induced_factor() * (weight * load_factor) ** 2 / dynamic_area
    values = (drag - induced_drag) / dynamic_area
    return np.median(values), np.percentile(values, [10, 90]), np.count_nonzero(mask)


def fit_propeller(data, mass, parasitic_cd):
    rpm, esc_names = forward_rpm(data)
    mask = (fixed_wing_mask(data) & (data['airspeed'] < 23) & (data['throttle'] > 45) &
            (np.abs(data['roll']) < 12) & (np.abs(data['acceleration']) < 0.15))
    parasitic, induced = drag_components(mass, data['airspeed'][mask], data['roll'][mask], parasitic_cd)
    target_thrust = (parasitic + induced + mass * GRAVITY * data['climb_rate'][mask] /
                     data['airspeed'][mask] + mass * data['acceleration'][mask])
    rpm_fit = rpm[mask] / 1000.0
    airspeed = data['airspeed'][mask]
    design = np.column_stack((rpm_fit ** 2, -rpm_fit * airspeed))
    coefficients = np.linalg.lstsq(design, target_thrust, rcond=None)[0]
    return coefficients, esc_names, np.count_nonzero(mask)


def infer_payload_drag(data, mass, propeller_coefficients):
    rpm, esc_names = forward_rpm(data)
    mask = (fixed_wing_mask(data) & (data['airspeed'] < 23) & (data['throttle'] > 45) &
            (np.abs(data['roll']) < 12) & (np.abs(data['climb_rate']) < 0.35) &
            (np.abs(data['acceleration']) < 0.15))
    rpm_fit = rpm[mask] / 1000.0
    airspeed = data['airspeed'][mask]
    thrust = propeller_coefficients[0] * rpm_fit ** 2 - propeller_coefficients[1] * rpm_fit * airspeed
    _, induced = drag_components(mass, airspeed, data['roll'][mask], 0.0)
    dynamic_area = 0.5 * RHO0 * airspeed ** 2 * WING_AREA
    parasitic_cd = (thrust - mass * GRAVITY * data['climb_rate'][mask] / airspeed -
                    mass * data['acceleration'][mask] - induced) / dynamic_area
    return np.median(parasitic_cd), np.percentile(parasitic_cd, [10, 90]), esc_names, np.count_nonzero(mask)


def fit_rpm_current(data):
    rpm, esc_names = forward_rpm(data)
    mask = (fixed_wing_mask(data) & (data['airspeed'] < 23) & (np.abs(data['roll']) < 12) &
            (np.abs(data['climb_rate']) < 0.35) & (np.abs(data['acceleration']) < 0.15))
    x = rpm[mask] / 6000.0
    observed = data['current'][mask]

    best = None
    for exponent in np.linspace(0.5, 10.0, 381):
        power = np.maximum(x, 0) ** exponent
        design = np.column_stack((power, np.ones_like(power)))
        parameters = np.linalg.lstsq(design, observed, rcond=None)[0]
        for _ in range(8):
            residual = design @ parameters - observed
            scale = max(0.2, 1.4826 * np.median(np.abs(residual - np.median(residual))))
            weights = np.minimum(1.0, scale / np.maximum(np.abs(residual), 1.0e-9))
            parameters = np.linalg.lstsq(design * weights[:, None], observed * weights, rcond=None)[0]
        amplitude, fixed = parameters
        if amplitude < 0 or amplitude > 200 or fixed < 0 or fixed > 10:
            continue
        residual = amplitude * power + fixed - observed
        scale = max(0.2, 1.4826 * np.median(np.abs(residual - np.median(residual))))
        score = np.mean(np.where(np.abs(residual) <= scale, 0.5 * residual ** 2,
                                 scale * (np.abs(residual) - 0.5 * scale)))
        if best is None or score < best[0]:
            best = (score, amplitude, exponent, fixed)
    if best is None:
        raise RuntimeError('could not fit baseline current against forward-motor RPM')
    return np.asarray(best[1:]), esc_names, np.count_nonzero(mask)


def payload_current_offset(data, current_parameters):
    rpm, esc_names = forward_rpm(data)
    mask = (fixed_wing_mask(data) & (data['airspeed'] < 23) & (np.abs(data['roll']) < 12) &
            (np.abs(data['climb_rate']) < 0.35) & (np.abs(data['acceleration']) < 0.15))
    x = rpm[mask] / 6000.0
    predicted = current_parameters[0] * x ** current_parameters[1] + current_parameters[2]
    residual = data['current'][mask] - predicted
    return np.median(residual), np.percentile(residual, [10, 90]), esc_names, np.count_nonzero(mask)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline_log')
    parser.add_argument('payload_log')
    parser.add_argument('--baseline-mass', type=float, required=True, help='camera-free all-up mass in kg')
    parser.add_argument('--payload-mass', type=float, required=True, help='payload-equipped all-up mass in kg')
    args = parser.parse_args()

    baseline = resample(load_log(args.baseline_log))
    payload = resample(load_log(args.payload_log))
    baseline_cd, baseline_range, glide_count = baseline_parasitic_cd(baseline, args.baseline_mass)
    propeller, baseline_esc, prop_count = fit_propeller(baseline, args.baseline_mass, baseline_cd)
    payload_cd, payload_range, payload_esc, payload_count = infer_payload_drag(payload, args.payload_mass, propeller)
    current_fit, current_esc, current_count = fit_rpm_current(baseline)
    current_offset, current_range, payload_current_esc, offset_count = payload_current_offset(payload, current_fit)

    baseline_drag = sum(drag_components(args.baseline_mass, 20.0, 0.0, baseline_cd))
    payload_drag = sum(drag_components(args.payload_mass, 20.0, 0.0, payload_cd))
    print('baseline mass:           %.3f kg' % args.baseline_mass)
    print('payload mass:            %.3f kg (%+.3f kg)' %
          (args.payload_mass, args.payload_mass - args.baseline_mass))
    print('forward ESCs:            baseline %s, payload %s' % (','.join(baseline_esc), ','.join(payload_esc)))
    print('baseline parasitic Cd:   %.4f (p10-p90 %.4f..%.4f, %u samples)' %
          (baseline_cd, baseline_range[0], baseline_range[1], glide_count))
    print('payload parasitic Cd:    %.4f (p10-p90 %.4f..%.4f, %u samples)' %
          (payload_cd, payload_range[0], payload_range[1], payload_count))
    print('equivalent drag area:    %+.4f m^2' % ((payload_cd - baseline_cd) * WING_AREA))
    print('drag at 20 m/s:         %.2f -> %.2f N (%+.2f N, %+.1f%%)' %
          (baseline_drag, payload_drag, payload_drag - baseline_drag,
           100 * (payload_drag / baseline_drag - 1)))
    print('propeller fit:           T = %.6f*(RPM/1000)^2 - %.6f*(RPM/1000)*EAS (%u samples)' %
          (propeller[0], propeller[1], prop_count))
    print('baseline current fit:    I = %.3f*(RPM/6000)^%.3f + %.3f (%u samples, ESCs %s)' %
          (current_fit[0], current_fit[1], current_fit[2], current_count, ','.join(current_esc)))
    print('payload fixed current:  %+.3f A (p10-p90 %+.3f..%+.3f, %u samples, ESCs %s)' %
          (current_offset, current_range[0], current_range[1], offset_count, ','.join(payload_current_esc)))


if __name__ == '__main__':
    main()
