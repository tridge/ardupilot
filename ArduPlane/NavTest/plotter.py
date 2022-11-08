#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.animation as animation
import sys

N = 41
Zmin = 0.0
Zmax = 1.0
fname = sys.argv[1]

def load_data():
    '''load mesh file'''
    global Zmax, Zmin, N
    data = []
    f = open(fname,'r')
    while True:
        try:
            tstamp = f.readline()
        except Exception:
            return data
        a = np.zeros((N,N))
        for y in range(N):
            xdata = f.readline().split()
            if len(xdata) != N:
                return data
            for x in range(N):
                a[y,x] = float(xdata[x])
                Zmax = max(a[y,x],Zmax)
                Zmin = min(a[y,x],Zmin)
        data.append(a)

gdata = load_data()

def update_plot(frame_number):
    global plot
    print('frame_number: ', frame_number, len(plot))
    plot[0].remove()
    plot[0] = ax.plot_surface(x, y, gdata[frame_number], cmap="magma")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

nmax=len(gdata)
x = np.linspace(10*(-N//2),10*(N//2),N)
x, y = np.meshgrid(x, x)

plot = [ax.plot_surface(x, y, gdata[0], cmap="magma")]
ax.set_zlim(Zmin,Zmax)
animate = animation.FuncAnimation(fig, update_plot, nmax, interval=500, fargs=())
plt.show()
