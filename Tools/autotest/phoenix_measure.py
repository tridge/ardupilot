#!/usr/bin/env python3
"""Measure fixed-wing phase performance from an ArduPilot BIN log.

Resamples every channel onto a common uniform time base from each message's own
timestamp, rather than pairing on 'last value seen' - the latter silently mixes
samples from different times when message rates differ, which made VTOL climb-out
show up as 'cruise'.
"""
import sys, numpy as np
from pymavlink import mavutil

def load(path):
    m = mavutil.mavlink_connection(path)
    ch = {}
    def add(k, t, v):
        ch.setdefault(k, ([], []))
        ch[k][0].append(t); ch[k][1].append(v)
    while True:
        x = m.recv_match(type=['CTUN','QTUN','BAT','ARSP','BARO','ATT'])
        if x is None: break
        t = getattr(x, '_timestamp', 0)
        if not t: continue
        d = x.to_dict(); ty = x.get_type()
        if   ty=='CTUN': add('thr', t, d.get('ThO', 0))
        elif ty=='QTUN': add('qthr', t, d.get('ThO', 0))
        elif ty=='ARSP' and d.get('I',0)==0: add('as', t, d.get('Airspeed',0))
        elif ty=='BARO' and d.get('I',0)==0: add('alt', t, d.get('Alt',0))
        elif ty=='BAT'  and d.get('Inst',d.get('I',0))==0:
            add('I', t, d.get('Curr',0)); add('V', t, d.get('Volt',0))
            add('rem', t, d.get('RemPct',-1)); add('used', t, d.get('CurrTot',0))
    return ch

def resample(ch, dt=0.5):
    # Window comes from the ALWAYS-PRESENT channels only. QTUN is logged only
    # while the quadplane is active, so intersecting its range would truncate
    # the window to the VTOL climb-out and throw the whole cruise away.
    core = [k for k in ('thr','as','alt','I','V') if k in ch and len(ch[k][0])>10]
    opt  = [k for k in ('qthr','rem','used')      if k in ch and len(ch[k][0])>10]
    t0 = max(min(ch[k][0]) for k in core); t1 = min(max(ch[k][0]) for k in core)
    grid = np.arange(t0, t1, dt)
    out = {k: np.interp(grid, ch[k][0], ch[k][1]) for k in core}
    for k in opt:
        # outside its logged range, hold the nearest value (qthr -> 0 once VTOL stops)
        out[k] = np.interp(grid, ch[k][0], ch[k][1],
                           left=ch[k][1][0], right=0.0 if k=='qthr' else ch[k][1][-1])
    out['t'] = grid
    return out

def report(d, label, dt=0.5):
    t, alt = d['t'], d['alt']
    # climb rate against TIME, smoothed over ~20 s
    w = max(3, int(20/dt))
    vs = np.gradient(alt, t)
    vs = np.convolve(vs, np.ones(w)/w, mode='same')
    fw = (d['as'] > 15) & (alt > 80)
    if 'qthr' in d: fw &= (d['qthr'] < 0.10)      # lift motors idle => forward flight
    print(f"\n=== {label} ===")
    print(f"  {fw.sum()*dt/60:.1f} min of fixed-wing flight")
    if 'rem' in d:
        print(f"  battery: RemPct {d['rem'][0]:.0f} -> {d['rem'][-1]:.0f}%  ({d['rem'][0]-d['rem'][-1]:.0f}% used)")
    sc = 100.0 if np.nanmax(d['thr']) <= 1.01 else 1.0
    thrp = d['thr'] * (100.0 if np.nanmax(d['thr']) <= 1.01 else 1.0)
    # The descent is bimodal - idle glides mixed with powered descents. Averaging
    # them yields a throttle/sink pair the aircraft never actually flew, so they
    # are reported separately. The idle glide additionally gives a direct
    # zero-thrust L/D (V/Vz) with no efficiency assumption.
    for nm, mk in (("cruise  ", fw & (np.abs(vs) < 0.3)),
                   ("climb   ", fw & (vs > 1.0)),
                   ("glide   ", fw & (vs < -0.5) & (thrp < 3.0)),
                   ("pwr-desc", fw & (vs < -1.0) & (thrp > 40.0))):
        if mk.sum() < 10:
            print(f"  {nm}: {mk.sum()} samples"); continue
        extra = ""
        if nm.startswith("glide"):
            extra = f"  L/D {(d['as'][mk]/-vs[mk]).mean():5.2f}"
        print(f"  {nm}: {mk.sum()*dt/60:5.1f} min  thr {d['thr'][mk].mean()*sc:5.1f}%  "
              f"as {d['as'][mk].mean():5.2f} m/s  I {d['I'][mk].mean():6.2f} A  "
              f"V {d['V'][mk].mean():5.2f}  Vz {vs[mk].mean():+5.2f}{extra}")

if __name__ == '__main__':
    for p in sys.argv[1:]:
        report(resample(load(p)), p.split('/')[-1])
