#!/usr/bin/python
import sys
import matplotlib.pyplot as plt
from subprocess import call

def drawx(x, y):
    x = float(x);
    y = float(y);
    line1, = plt.plot([x-30, x+30], [y+30, y-30], linewidth=2)
    lcolor = line1.get_color()
    plt.plot([x+30, x-30], [y+30, y-30], linewidth=2, color=lcolor)
    
def readtest():
    probfile = open("data/debug.out", "r")

    cnt = int(0)
    for line in probfile:
        (x1, y1, x2, y2) = line.split()
        if cnt == 0:
            drawx(x1, y1);
            drawx(x2, y2);
        else:
            plt.plot([x1, x2], [y1, y2], linewidth=5)#, color = "black")
        cnt = cnt + 1
    print cnt

plt.figure(figsize=(100, 100))

infile = open("data/road/ccsegment.dat", "r")
cnt = int(0);

for line in infile:
    (x1, y1, x2, y2, width, sid) = line.split()

    xxx = (int(width)+3)/4
    yyy = int(width);
    xxx = 3
    plt.plot([x1, x2], [y1, y2], linewidth=xxx, color = "black")


#readtest()

savepath="./data/png/debug.png"
plt.savefig(savepath)
call(["shotwell", savepath])
