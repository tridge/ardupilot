#!/usr/bin/env python
'''
decode an Ikalogic CSV export file as PWM values. Used for
testing output values on PWM channels
'''

import csv, sys
import matplotlib.pyplot as plt
import numpy as np;

filename = sys.argv[1]

c = open(filename, 'r')
data = csv.reader(c,delimiter=';')

pulse_start = []
prev_values = []
pwm = []
nchannels = 0
x = []
y = []
min_frame_gap = 1e6
min_gap_t = 0
trailing_edge = 0
last_frame = 0
frame_start = 0
frame_interval = 0
min_frame_interval = 1e6
max_frame_interval = 0
min_interval_t = 0
max_interval_t = 0

for row in data:
    # skip header
    if row[0].startswith('Time'):
        continue
    changed = False

    if len(row) > nchannels+1:
        nchannels = len(row)-1
        while len(pwm) < nchannels:
            pwm.append(0)
            pulse_start.append(0)
            prev_values.append(0)

    # time in seconds
    t = float(row[0])

    # current value of each channel
    values = [int(row[i]) for i in range(1,len(row))]
    
    # channel uch is the uart output
    uch = 3
    # pwm channel range start
    pwm0 = 0
    # look for start of frame at each rising edge
    if values[uch] == 1 and prev_values[uch] == 0 and trailing_edge != 0:
        # require interframe gap of at least 12 bit intervals)
        if (t - trailing_edge) > .000120:
            frame_interval = t - frame_start
            if frame_interval > .003 and frame_interval < min_frame_interval: 
                min_frame_interval = frame_interval
                min_interval_t = t
            if (frame_interval < .008) and (frame_interval > max_frame_interval): 
                max_frame_interval = frame_interval
                max_interval_t = t
            frame_gap = t - trailing_edge
            frame_start = t
#             print("frame_start: %.6f, gap: %.6f" % (frame_start, frame_gap))
            if frame_gap < min_frame_gap:
                min_frame_gap = frame_gap
                min_gap_t = t
    
    # record time of last trailing edge
    if values[uch] == 0 and prev_values[uch] == 1:
        trailing_edge = t
        
    prev_values[uch] = values[uch]
                
    pwmN = pwm0+nchannels-1
    channelset = range(pwm0,pwmN)
    for c in channelset:
        if values[c] == 0 and prev_values[c] == 1 and pulse_start[c] != 0:
            pulse = t - pulse_start[c]
            if pulse < .003:
                pwm[c] = pulse
                changed = True
        if values[c] == 1 and prev_values[c] == 0:
            pulse_start[c] = t
        prev_values[c] = values[c]
        
    if changed:
        sys.stdout.write("%.7f" % t)
        for c in channelset:
            sys.stdout.write(" %.1f" % (1e6*pwm[c]))
        sys.stdout.write("\n")
            
            
        x.append(t)
        y.append(pwm[pwm0:pwmN])

sys.stdout.write("min_frame_gap: %.6f at %.6f\n" % (min_frame_gap, min_gap_t))
sys.stdout.write("min_frame_interval: %.6f, at: %.6f\n" % (min_frame_interval, min_interval_t))
sys.stdout.write("max_frame_interval: %.6f, at: %.6f\n" % (max_frame_interval, max_interval_t))

xa = np.array(x)
ya = np.array(y)
plt.figure(1)
plt.ion()
plt.plot(xa, ya)
plt.grid()
plt.legend(['1','2','3'])

plt.show()

