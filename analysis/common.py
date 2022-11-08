#!/usr/bin/env python

import re
import os

r = re.compile(r'Message : (ArduCopter\s\S+)\s\((\w+)\)')

def add_version(f, bin_file):
    '''add version information if available'''
    msg_file = bin_file + '.msg'
    parm_file = bin_file + '.parm'

    if os.path.exists(parm_file):
        parms = open(parm_file, 'r').read()
    else:
        parms = None

    # note if we are using LOG_REPLAY as that greatly affects analysis
    if parms.find("LOG_REPLAY      0.000000") != -1:
        f.write('LOG_REPLAY: 0<br>\n')
    if parms.find("LOG_REPLAY      1.000000") != -1:
        f.write('LOG_REPLAY: 1<br>\n')

    # first see if we have version in msg file
    if os.path.exists(msg_file):
        msgs = open(msg_file, 'r').read()
        if msgs.find("ArduCopter") != -1:
            m = r.search(msgs)
            if m is not None:
                version = m.group(1)
                githash = m.group(2)
                vstr = '%s (%s)' % (version, githash)
                f.write('Version: <a href="https://github.com/ardupilot/ardupilot/commit/%s" target="_blank">%s</a><br>\n' % (githash, vstr))
                return

    # fallback to parm file
    if parms is not None:
        if parms.find("RC_FEEL_RP") != -1:
            # definately an old NuttX build
            f.write('Version: Unknown NuttX<br>\n')
            return

        if parms.find("BRD_SD_SLOWDOWN") != -1:
            # definately ChibiOS
            f.write('Version: Unknown ChibiOS<br>\n')
            return

    # we don't know
    f.write('Version: Unknown<br>\n')

if __name__ == '__main__':
    import sys
    for f in sys.argv[1:]:
        add_version(sys.stdout, f)
