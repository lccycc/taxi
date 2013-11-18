#!/usr/bin/python
import sys
import matplotlib.pyplot as plt
from subprocess import call
plt.figure(figsize=(100, 100))

#infile = open("data/speedsegment.dat", "r")
infile = open("data/road/ccsegment.dat", "r")
cnt = int(0);
for line in infile:
    (x1, y1, x2, y2, width) = line.split()
    plt.plot([x1, x2], [y1, y2], linewidth=(int(width)+3)/4, color = "black")
    cnt += 1
    if cnt%10000 == 0:
        print cnt

infile = open("data/road/restpng.dat", "r")
cnt = int(0);
for line in infile:
    (x1, y1, x2, y2, width) = line.split()
    plt.plot([x1, x2], [y1, y2], color = "red")
    cnt += 1
    if cnt%10000 == 0:
        print cnt
    if cnt>1000000:
        break

savepath="./data/png/restpnt_tmp.png"
plt.savefig(savepath)
call(["shotwell", savepath])
