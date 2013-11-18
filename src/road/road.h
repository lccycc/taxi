#ifndef ROAD_H
#define ROAD_H

#include "cross.h"

class Road{
public:
	vector<Point> pts;
	double width;
};

class Prob;

class RoadMap{
public:
	vector<Road> roads;
	UMAP<LL, UMAP<LL, vector<pair<LL, bool> > > > neib;
	UMAP<LL, UMAP<LL, vector<pair<LL, bool> > > > fpa;

	void ifneib(Point &p1, Point &p2, LL sid, CrossCounter &cc, bool ifpvt);
	vector<pair<LL, bool> > findpath(CrossCounter &cc, LL sid1, LL sid2);
	void findpathAll(CrossCounter &cc);
	void buildConnect(CrossCounter &cc);
	void buildConnect(CrossCounter &cc, bool simple);
	void build(UMAP<LL, Segment> &segmap);
	void outputroad();
};
#include "prob/prob.h"

#endif
