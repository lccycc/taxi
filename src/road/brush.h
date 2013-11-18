#ifndef BRUSH_H
#define BRUSH_H

#include "include/head.h"
#include "include/point.h"
#include "include/timestamp.h"
#include "taxi/taxi.h"
#include "cross.h"

class Brush{
public:
	string name;

	vector<TaxiPoint> rest;

	Brush(string _name);
	void getrest(Taxi &t, CrossCounter &c);
	void getrest(TaxiTrace &tr, CrossCounter &c);
	void outputrestpnt();
	void save();
	void load();
};
#endif
