#ifndef POINT_CPP
#define POINT_CPP
#include "point.h"
#include <cmath>
double sqr(double a){
	return a*a;
}
Point::Point(){
	x = y = 0;
}
Point::Point(IPoint a){
	x = a.x;
	y = a.y;
}
double dis(Point a, Point b){
	return sqrt(sqr(a.x-b.x)+sqr(a.y-b.y));
}
Point::Point(double toward){
	x = cos(toward*M_PI/180);
	y = sin(toward*M_PI/180);
}
Point::Point(double _x, double _y){
	x = _x;
	y = _y;
}
Point operator +(Point a, Point b){
	a.x+=b.x;
	a.y+=b.y;
	return a;
}
Point operator -(Point a, Point b){
	a.x-=b.x;
	a.y-=b.y;
	return a;
}
Point operator *(Point a, double d){
	a.x*=d;
	a.y*=d;
	return a;
}
Point operator *(double d, Point a){
	a.x*=d;
	a.y*=d;
	return a;
}
Point operator /(Point a, double d){
	a.x/=d;
	a.y/=d;
	return a;
}
bool operator <(Point a, Point b){
	return a.x<b.x || (a.x == b.x && a.y<b.y);
}

IPoint::IPoint(){
	x = y = 0;
}
IPoint::IPoint(int _x, int _y){
	x = _x;
	y = _y;
}
IPoint operator *(IPoint a, double b){
	a.x *= b;
	a.y *= b;
	return a;
}
IPoint operator *(double b, IPoint a){
	a.x *= b;
	a.y *= b;
	return a;
}
IPoint operator /(IPoint a, double b){
	a.x /= b;
	a.y /= b;
	return a;
}
bool operator <(IPoint a, IPoint b){
	return a.x<b.x || (a.x==b.x && a.y<b.y);
}
double dis(IPoint a, IPoint b){
	return sqrt(sqr(a.x-b.x)+sqr(a.y-b.y));
}
double dis(Point a, IPoint b){
	return sqrt(sqr(a.x-b.x)+sqr(a.y-b.y));
}
double dis(IPoint a, Point b){
	return sqrt(sqr(a.x-b.x)+sqr(a.y-b.y));
}

double angledis(double a, double b){
	a = angleform(a);
	b = angleform(b);
	if (b>a) swap(a, b);
	return min(fabs(a-b), fabs(a-b-360));
}
double angleform(double a){
	while (a>=360) a-=360;
	while (a<0) a+=360;
	return a;
}
double justtoward(double toward, double justify){
	toward = angleform(toward);
	justify = angleform(justify);
	if (angledis(toward, justify) > angledis(angleform(toward+180), justify)){
		toward = angleform(toward + 180);
	}
	return toward;
}
double disls(Point p, Point l, Point r){
	double a = dis(l, r);
	double b = dis(l, p);
	if (a<1e-4) return b;
	double c = dis(r, p);
	if (b*b>a*a+c*c) return c;
	if (c*c>a*a+b*b) return b;
	double e = (a+b+c)/2;
	double area = sqrt(e*(e-a)*(e-b)*(e-c));
	double h = area*2/a;
	return h;
}
Point rotate(Point a, double toward){
	toward *= M_PI/180;
	double x = a.x * cos(toward) - a.y * sin(toward);
	double y = a.x * sin(toward) + a.y * cos(toward);
	return Point(x, y);
}
#endif
