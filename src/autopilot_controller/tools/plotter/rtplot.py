# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.lines as line
import matplotlib
import math
import random
import time

import threading
import socket
import struct
import os

fig = plt.figure()

N = 6000

e = [0] * N  # error distance
ff = [0] * N  # feedforward control
fb = [0] * N  # feedback control
da = [0] * N  # damping control
s = [0] * N  # steering
Ucur = [0] * N  # cur_speed
Utar = [0] * N  # tar_speed

axis_t = np.linspace(0, 60, num=N)

U_ax = plt.subplot(311, xlim=(0, 60), ylim=(0, 80))
U_ax.set_title("Speed Controller")
U_ax_l = plt.axhline(linewidth=0.5, color='r')

e_ax = plt.subplot(312, xlim=(0, 60), ylim=(-4, 4))
e_ax.set_title("Real Time Error")
e_ax_l = plt.axhline(linewidth=0.5, color='r')

x_ax = plt.subplot(313, xlim=(0, 60), ylim=(-550, 550))
x_ax.set_title("Real Time Steering")
x_ax_l = plt.axhline(linewidth=0.5, color='r')

Ucur_line = line.Line2D([], [], linewidth='2', color='blue')
Utar_line = line.Line2D([], [], linewidth='2', color='r')

e_line = line.Line2D([], [], linewidth='2')

ff_line = line.Line2D([], [], linewidth='1', color='cyan')
fb_line = line.Line2D([], [], linewidth='1', color='magenta')
da_line = line.Line2D([], [], linewidth='1', color='blue')
s_line = line.Line2D([], [], linewidth='2', color='black')


def init():
    U_ax.add_line(Ucur_line)
    U_ax.add_line(Utar_line)

    e_ax.add_line(e_line)

    x_ax.add_line(ff_line)
    x_ax.add_line(fb_line)
    x_ax.add_line(da_line)
    x_ax.add_line(s_line)

    return Ucur_line, Utar_line, e_line, ff_line, fb_line, da_line, s_line,


def update(i):
    Ucur_line.set_xdata(axis_t)
    Utar_line.set_xdata(axis_t)
    Ucur_line.set_ydata(Ucur)
    Utar_line.set_ydata(Utar)

    e_line.set_xdata(axis_t)
    e_line.set_ydata(e)

    ff_line.set_xdata(axis_t)
    fb_line.set_xdata(axis_t)
    da_line.set_xdata(axis_t)
    s_line.set_xdata(axis_t)
    ff_line.set_ydata(ff)
    fb_line.set_ydata(fb)
    da_line.set_ydata(da)
    s_line.set_ydata(s)

    return Ucur_line, Utar_line, e_line, ff_line, fb_line, da_line, s_line,


ani = animation.FuncAnimation(fig, update,
                              init_func=init,
                              frames=1,
                              interval=125,
                              blit=True,
                              )

fig.legend((Ucur_line, Utar_line, e_line, ff_line, fb_line, da_line, s_line),
           ('Ucur', 'Utar', 'e', 'ff', 'fb', 'da', 's'),
           'center left')
fig.show()


# 数据获取和打印线程
def data_thread():
    address = ('', 30005)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(address)

    while True:
        data, addr = sock.recvfrom(80)
        fft, Pt, It, dat, st, et, cur_speed, tar_speed, accelerate_percentage, brake_percentage \
            = struct.unpack("dddddddddd", data)
        os.system('clear')
        print("\n=========================================================\n")
        print("cur_speed=% 2.02lf\ttar_speed=% 2.02lf" % (cur_speed, tar_speed))
        print("e=%- 3.02lf ff=%- 3.02f P=%- 4.02lf I=%- 4.02lf da=%- 4.02lf steer=%-.02lf" \
              % (et, fft, Pt, It, dat, st))
        print("accelerate_percentage=%- 4.02lf" % (accelerate_percentage))
        print("brake_percentage=%- 3.02lf" % (brake_percentage))
        print("\n=========================================================\n")

        Ucur.pop(0)
        Ucur.append(cur_speed)
        Utar.pop(0)
        Utar.append(tar_speed)

        e.pop(0)
        e.append(et)

        ff.pop(0)
        ff.append(fft)
        fb.pop(0)
        fb.append(Pt + It)
        da.pop(0)
        da.append(dat)
        s.pop(0)
        s.append(st)

    sock.close()


t = threading.Thread(target=data_thread, args=())

t.daemon = True
t.start()

input("wait")
