#ifndef POINT_H
#define POINT_H
#include "head.h"

double sqr(double a);

class IPoint;

class Point{
public:
	double x, y;
	Point();
	Point(double _x, double _y);
	Point(double toward);
	Point(IPoint);
};
double dis(Point a, Point b);
Point operator +(Point a, Point b);
Point operator -(Point a, Point b);
Point operator *(Point a, double d);
Point operator *(double d, Point a);
Point operator /(Point a, double d);
bool operator <(Point a, Point b);

class IPoint{
public:
	int x, y;
	IPoint();
	IPoint(int _x, int _y);
};
bool operator <(IPoint a, IPoint b);
IPoint operator *(IPoint a, double b);
IPoint operator *(double b, IPoint a);
IPoint operator /(IPoint a, double b);
double dis(IPoint a, IPoint b);

double dis(IPoint a, Point b);
double dis(Point a, IPoint b);

double justtoward(double toward, double justify);
double angledis(double a, double b);
double angleform(double a);
double disls(Point p, Point l, Point r);
Point rotate(Point a, double toward);

#endif
