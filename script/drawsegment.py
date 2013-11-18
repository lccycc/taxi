#!/usr/bin/python
import sys
import matplotlib.pyplot as plt
from subprocess import call
plt.figure(figsize=(100, 100))

#infile = open("data/speedsegment.dat", "r")
infile = open("data/small.dat", "r")
#infile = open("xxx", "r")
cnt = int(0)
for line in infile:
    (x1, y1, x2, y2) = line.split()
    plt.plot([x1, x2], [y1, y2], linewidth=1, color = "black")
    cnt += 1
    if cnt % 10000 == 0:
        print cnt

#plt.plot([4400*0.39817+2408, 4520*0.39817+2408], [4400, 4520], linewidth=1, color = "black")
savepath="./data/png/beijing_smalldataset_tmp.png"
plt.savefig(savepath)
call(["shotwell", savepath])
