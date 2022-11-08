#!/usr/bin/env python

import sys, random, os, time, multiprocessing, subprocess
from pymavlink import mavutil
from MAVProxy.modules.lib import grapher
import common

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--newer", action='store_true')
parser.add_argument("--parallel", default=6, type=int)
parser.add_argument("--mission", type=int, default=0)
parser.add_argument("files", type=str, nargs='+', help="input files")
args = parser.parse_args()

files = list(dict.fromkeys(args.files))
nfiles = len(files)

graphed = set()

graphs = [
    ('Speed and Height', 'GPS.Spd<ground_speed(m/s)> XKF5.HAGL{XKF5.C>=100}:2<HAGL(m)> -XKF5.offset{XKF5.C>=100}:2<EKF_terrain_height(m)>'),
    ('Dead-Reckoning Position Error', 'distance_two(GPS,NTDR[0]){GPS.TimeUS<=NTDR[0].TimeUS}<IMU1_error(m)> distance_two(GPS,NTDR[1]){GPS.TimeUS<=NTDR[1].TimeUS}<IMU2_error(m)> distance_two(GPS,NTDR[2]){GPS.TimeUS<=NTDR[2].TimeUS}<IMU3_error(m)> distance_two(GPS,NTDR[100]){GPS.TimeUS<=NTDR[100].TimeUS}<IMU1_replay_error(m)> distance_two(GPS,NTDR[101]){GPS.TimeUS<=NTDR[101].TimeUS}<IMU2_replay_error(m)> distance_two(GPS,NTDR[102]){GPS.TimeUS<=NTDR[102].TimeUS}<IMU3_replay_error(m)>'),
    ('TNAV Position Error', 'distance_two(GPS,NTMD[0]){GPS.TimeUS<=NTMD[0].TimeUS}<error(m)> sqrt(NTK3[0].SPN**2+NTK3[0].SPE**2)<IMU1_estimated_1-sigma_error(m)> sqrt(NTK3[1].SPN**2+NTK3[1].SPE**2)<IMU2_estimated_1-sigma_error(m)> sqrt(NTK3[2].SPN**2+NTK3[2].SPE**2)<IMU3_estimated_1-sigma_error(m)>'),
    ('Replay TNAV Position Error', 'distance_two(GPS,NTMD[100]){GPS.TimeUS<=NTMD[100].TimeUS}<error(m)> sqrt(NTK3[100].SPN**2+NTK3[100].SPE**2)<IMU1_estimated_1-sigma_error(m)> sqrt(NTK3[101].SPN**2+NTK3[101].SPE**2)<IMU2_estimated_1-sigma_error(m)> sqrt(NTK3[102].SPN**2+NTK3[102].SPE**2)<IMU3_estimated_1-sigma_error(m)>'),
    ('Replay Combined Position Error', 'distance_two(GPS,NTMD[100]){GPS.TimeUS<=NTMD[100].TimeUS}<TNAV_error(m)> sqrt(NTK3[100].SPN**2+NTK3[100].SPE**2)<IMU1_TNAV_error_estimate(m)> sqrt(NTK3[101].SPN**2+NTK3[101].SPE**2)<IMU2_TNAV_error_estimate(m)> sqrt(NTK3[102].SPN**2+NTK3[102].SPE**2)<IMU3_TNAV_error_estimate(m)> distance_two(GPS,NTDR[100]){GPS.TimeUS<=NTDR[100].TimeUS}:2<IMU1_DR_error(m)> distance_two(GPS,NTDR[101]){GPS.TimeUS<=NTDR[101].TimeUS}:2<IMU2_DR_error(m)> distance_two(GPS,NTDR[102]){GPS.TimeUS<=NTDR[102].TimeUS}:2<IMU3_DR_error(m)>'),
    ('Position Error Comparison', 'distance_two(GPS,NTMD[0]){GPS.TimeUS<=NTMD[0].TimeUS}<original_error(m)> distance_two(GPS,NTMD[100]){GPS.TimeUS<=NTMD[100].TimeUS}<replayed_error(m)>'),
    ('Wind Speed & Direction', 'sqrt(XKF2[0].VWN**2+XKF2[0].VWE**2){GPS.Spd>3}<wind_speed_(m/s)> wrap_360(degrees(atan2(-XKF2[0].VWE,-XKF2[0].VWN))){GPS.Spd>3}:2<wind_direction_(deg)>'),
]

def graph_one(mlog, title, expression, filename):
    '''create one graph'''
    print("Graphing %s to %s" % (title, filename))
    mg = grapher.MavGraph()
    mg.add_mav(mlog)
    for e in expression.split():
        mg.add_field(e)
    mg.set_title(title)
    mg.process([],[],0)
    mg.set_grid(True)
    mg.show(1, output=filename)

def script_path():
    '''path of this script'''
    import inspect
    return os.path.abspath(inspect.getfile(inspect.currentframe()))

def process_one(fname):
    '''process one file'''
    html = fname + ".html"
    print("Graphing %s %u/%u" % (fname, len(graphed), len(files)))

    tmp = html+".tmp"+str(random.randint(0,100000))
    tmpg = tmp+".png"
    try:
        os.unlink(tmp)
    except Exception:
        pass
    try:
        os.unlink(tmpg)
    except Exception:
        pass

    bname = os.path.basename(fname)
    bname2 = bname[:8]
    dname = os.path.dirname(fname)
    f = open(tmp, "w")
    f.write("<html><head><title>Graphs of %s</title><body>\n" % bname)
    f.write("<h1>Graphs of log %s</h1>\n" % bname)
    f.write('Logfile: <a href="%s" target="_blank">%s</a><br>\n' % (bname, bname))
    f.write("<pre>")
    proc = subprocess.Popen("mavflighttime.py %s | egrep '^Flig|^Total.dist'" % fname, shell=True, stdout=subprocess.PIPE)
    f.write(str(proc.stdout.read()))
    f.write("</pre><p>")

    map_log = "%s/%s.bin" % (dname, bname2)
    map_img = "%s-map.png" % bname
    map_img2 = "%s-map2.png" % bname

    os.system("mavflightview.py --imagefile=%s/%s --types=GPS %s" % (dname, map_img, map_log))
    f.write('<hr><p><table><tr><td><img src="%s"></td><td><img\n' % (map_img))

    os.system("mavflightview.py --imagefile=%s/%s --types=NTRG --no-show-lines %s" % (dname, map_img2, map_log))
    f.write('<hr><p><table><tr><td><img src="%s"></td><td><img\n' % (map_img2))

    mlog = mavutil.mavlink_connection(fname)
    idx = 0
    for (title, expression) in graphs:
        idx += 1
        if os.path.exists(tmpg):
            os.unlink(tmpg)
        p = multiprocessing.Process(target=graph_one, args=(mlog, title, expression, tmpg))
        p.start()
        p.join()
        if os.path.exists(tmpg):
            os.rename(tmpg, "%s/%s-g%u.png" % (dname, bname, idx))
            png_name = "%s-g%u.png" % (bname, idx)
            f.write('<a href="%s"><img src="%s" width=1035 height=527></a><p>\n' % (png_name, png_name))
    f.write("</body>\n")
    f.close()
    if os.path.exists(tmp):
        try:
            os.unlink(html)
        except Exception:
            pass
        os.rename(tmp, html)

def is_done(fname):
    '''check if already done'''
    return False

files2 = []
for f in files:
    if os.path.getsize(f) == 0:
        continue
    if not is_done(f + ".html"):
        files2.append(f)
files = files2
nfiles = len(files2)

procs = []

while len(graphed) < nfiles:
    r = random.randint(0, nfiles-1)
    fname = files[r]
    if fname in graphed:
        continue
    graphed.add(fname)
    html = fname + ".html"
    if is_done(html):
        continue
    p = multiprocessing.Process(target=process_one, args=(fname,))
    p.start()
    procs.append(p)
    while len(procs) >= args.parallel:
        for p in procs:
            if p.exitcode is not None:
                p.join()
                procs.remove(p)
        time.sleep(0.1)
