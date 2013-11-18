#ifndef CROSS_H
#define CROSS_H
#include "include/head.h"
#include "include/point.h"
#include "include/timestamp.h"
#include "taxi/taxi.h"
class Cross{
public:
	double toward;
	double avertow;
	int counter;
	Cross();
	void overlap(double overtow);
	void overlap(double overtow, int weight);
};

class Segment{
public:
	/* XXX should change to LL */
	int id;
	Point p1, p2;
	double width;
	bool valid;

	Segment();
	Segment(Point _p1, Point _p2);
};

double getSegTw(Segment s);
double getPntPairTw(Point p1, Point p2);

class Line{
public:
	double a, b, c;
	Line();
	Line(double _a, double _b, double _c);
	Line(Point p, Point q);

	double f(Point p);
	double toward();
};
bool intersect(Line p, Line q, Point &s);

class CrossCounter{
public:
	const static LL baseX = 1000000;
	/*
	 * arguments:
	 */
	string name;

	int narrow;
	double towardfilter;
	double speedfilter;
	double smallrunsecond;
	bool regionLimit;
	Point regionLimit1, regionLimit2;
	bool gotaxiNightlimit;
	int avertowRange;
	bool avertowEnable;
	int bfspointRange;
	double bfspointHeartdis;
	double bfspointCounterlb;
	double getsegmentMidlenlb;
	double getsegmentMaxWidth;
	int overlapWidth;

	UMAP<LL, Cross> ccounter;
	UMAP<LL, Segment> segment;
	UMAP<LL, LL> xvisit;
	UMAP<LL, USET<LL> > gridsegxid;

	CrossCounter(string _name);
	LL getID(int x, int y);
	Cross& getCross(LL id);
	Cross& getCross(int x, int y);
	Cross& getCross(IPoint p);
	IPoint getIPoint(LL id);
	bool exist(int x, int y);

	bool sameline(TaxiPoint t1, TaxiPoint t2);
	static bool sameline(TaxiPoint t1, TaxiPoint t2, double towardfilter);
	vector<LL> footprint(TaxiPoint taxipt, double second);
	void overlap(TaxiPoint taxipt, double second, CrossCounter *notcover);

	void setArgument();
	void goTaxi(Taxi &t, CrossCounter *c);
	void goTaxiTrace(TaxiTrace &tr, CrossCounter *c);
	void countAvertow();
	void outputpoint();
	vector<IPoint> bfspoint(IPoint start, double &toward,
							UMAP<LL, LL> &visit, LL segid);
	void bfs();
	void getSegmentFromPoints(vector<IPoint> &ip, LL segid);
	void getSegmentFromPoints2(vector<IPoint> &ip, LL segid);
	void fishEat(UMAP<LL, Segment> &seg);
	void cleanunvalid();
	void merge(CrossCounter &c);
	void outputsegment(bool append, string filename);
	void outputTestPoint();

	static int towardSeg(Segment s);
	LL getSegmentIDForw(Point pt, double toward);
	LL getNearestSegmentID(Point pt, int fourtoward);
	LL getSegmentIDUntil(Point pt, int range);
	LL getSegmentID(Point pt);
	LL getSegmentID(Point pt, int range);
	USET<LL> getSegmentIDVect(Point pt, int range);
	USET<LL> getSegmentIDVectAll(Point pt, int exwidth);
	LL getGridxNearestSegmentID(Point pt);

	void init();
	void save();
	void load();
	void load(bool brief);
	void savecustom();
	void loadcustom();
};
#endif
