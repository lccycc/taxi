#ifndef TIMESTAMP_CPP
#define TIMESTAMP_CPP

#include "timestamp.h"
double TimeStamp::basetime = currentTime();
double TimeStamp::lasttime = 0;
double TimeStamp::currentTime(){
    struct timeval t;
    gettimeofday(&t, NULL);
    return t.tv_sec*1000000 + t.tv_usec;
}

string TimeStamp::gets(double p){
    p/=1000000;
    char s[10];
    int m = (int)(p/60);
    int h = (int)(m/60);
    if (h){
        sprintf(s, "%dh%02d:%02.02fs", h, m-h*60, p-m*60);
    }else
    if (m){
        sprintf(s, "%02d:%02.02fs", m, p-m*60);
    }else{
        sprintf(s, "%2.2fs", p-m*60);
    }
    return string(s);
}
string TimeStamp::gettime(){
    double t = currentTime();
    double p = t - lasttime - basetime;
    char tmps[10];
    sprintf(tmps, "%.2fs", p/1000000);
    string s = " " + string(tmps)+ " pass, now is "+gets(t-basetime)+"\n";
    lasttime = t - basetime;
    return s;
}
#endif
