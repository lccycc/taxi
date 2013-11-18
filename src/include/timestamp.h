#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include "head.h"
#include <sys/time.h>
class TimeStamp{
public:
    static double currentTime();
    static string gets(double p);
    static string gettime();
protected:
    static double lasttime, basetime;
};
#endif
