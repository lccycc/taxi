#!/usr/bin/python
import sys
import matplotlib.pyplot as plt
from subprocess import call
plt.figure(figsize=(100, 100))

infile = open("data/road/cctestpoint.dat", "r")
cnt = int(0)
for line in infile:
    (x1, y1, x2, y2) = line.split()
    plt.plot([x1, x2], [y1, y2], linewidth=1, color = "black")
    cnt += 1
    if cnt % 10000 == 0:
        print cnt

savepath="./data/png/beijing_testpoint.png"
plt.savefig(savepath)
call(["shotwell", savepath])
