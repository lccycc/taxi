#!/usr/bin/python
from random import choice
from random import random
import sys
import matplotlib.pyplot as plt
import matplotlib
from subprocess import call
plt.figure(figsize=(100, 100))

colors = [(1,1,1)] + [(random(),random(),random()) for i in xrange(255)]

#infile = open("data/speedsegment.dat", "r")
infile = open("data/road/roads.dat", "r")
cnt = int(0);
while True:
    line1 = infile.readline()
    if not line1: break
    line2 = infile.readline()
    if not line2: break
    xs = line1.split()
    ys = line2.split()

    plt.plot(xs, ys, linewidth=1, color=choice(colors))
    cnt += 1
    if cnt%10000 == 0:
        print cnt

savepath="./data/png/road_tmp.png"
plt.savefig(savepath)
call(["shotwell", savepath])
