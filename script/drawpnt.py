#!/usr/bin/python
import sys
import matplotlib.pyplot as plt
from subprocess import call

def drawx(x, y, x1, y1, x2, y2, idx):
    "x, y: point. (x1, y1)->(x2, y2): segment. id: testid"

    x = float(x);
    y = float(y);
    line1, = plt.plot([x-50, x+50], [y+50, y-50], linewidth=5)
    lcolor = line1.get_color()
    plt.plot([x+50, x-50], [y+50, y-50], linewidth=5, color=lcolor)
    
    plt.plot([x1, x2], [y1, y2], linewidth = 5, color = lcolor)

    plt.text(x+100, y+100, "%s"%idx, fontsize=100, color=lcolor)

def readtest():
    probfile = open("data/prob/probout.dat", "r")

    for line in probfile:
        (idx, weekday, hour, ypermin, x, y, segid, x1, y1, x2, y2, isright) = line.split()
        drawx(x, y, x1, y1, x2, y2, idx)


plt.figure(figsize=(100, 100))

#infile = open("data/road/probout.dat", "r")
infile = open("data/road/ccsegment.dat", "r")
cnt = int(0);

for line in infile:
    (x1, y1, x2, y2, width, sid) = line.split()
#    if float(x1) > 5000/3 or float(x1)<-5000/3 or float(y1)>5000/3 or float(y1)<-5000/3:
#            continue
#    if abs(float(x1) - -2703)>1000  or abs(float(y1)-2670)>1000:
#            continue
    xxx = (int(width)+3)/4
    yyy = int(width);
    xxx = 1
    plt.plot([x1, x2], [y1, y2], linewidth=xxx)#, color = "black")

    cnt += 1
    if cnt%10000 == 0:
        print cnt



#drawx(1700.01, -258.452, 1475.01, -273.506, 1737.84, -265.355, 0)
#drawx(-5469.48, -7189.66, -5498.63,-7374.12,-5447.17, -7146.55, 1)
readtest()
savepath="./data/png/ccpnt_tmp5000.png"
plt.savefig(savepath)
call(["shotwell", savepath])
