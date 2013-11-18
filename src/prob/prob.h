#ifndef PROB_H
#define PROB_H
#include "road/cross.h"
#include "road/road.h"
class Unit{
public:
	double full, empt;
	double v;
	double tdis;

	int ts[31][360];

	Unit();
	void overlap(double newv, int newtdis);
	void addfull(double newv, int newtdis);
	void addempt(double newv, int newtdis);
};

class ProbUnit{
public:
	Unit pvt, bac;
	int days;

	ProbUnit();
	void add(const TaxiPoint tp,
			bool ifpvt, bool isempty, double newv, int newtdis, bool ifaddday);
};

class ProbTestProfile{
public:
	double ypermin;
	Point pos; //have divide by narrow
	int weekday;
	int hour;
	LL segid;
	Segment seg;
	bool isright;
	ProbTestProfile();
};
class Prob{
public:
	CrossCounter &cc;
	string name;
	/* time id, segment id */
	UMAP<LL, ProbUnit> unitmap[4*24];
	UMAP<LL, pair<double, double> > cpvt[4*24];
	UMAP<LL, pair<double, double> > cbac[4*24];
	bool custommode;

	vector<ProbTestProfile> testresult;
	ProbTestProfile currentTest;

	/* arguments */
	int ifpvt_smalltoward;


	void buildcustom();
	void loadcustom();
	void term();

	Prob(CrossCounter &x, string probname);
	void setArgument();
	void countUnitDays();
	static bool ifpvt_must(Segment s, double toward);
	int ifpvt(Segment s, double toward);
	int getWeekDayID(int weekday);
	int getTimeID(int weekday, int hour);
	int getTimeID(const TaxiPoint tp);

	void overlap(const TaxiPoint tp, LL segid, int newtdis, bool ifaddday);
	USET<LL> footprint(const TaxiPoint tp, double second);
	void add(const TaxiPoint tp, Segment s, int newtdis, bool ifaddday);
	void add(const TaxiPoint tp, Segment s, int newtdis, bool ifpv, bool ifaddday);
	void add(const TaxiPoint tp, int newtdis, bool ifaddday);

	USET<LL> goPntPair(TaxiPoint tp, Point pto);
	void goTaxiTrace(TaxiTrace &tr);
	void goTaxiTrace(TaxiTrace &tr, RoadMap &r);
	void goTaxi(Taxi &t, RoadMap &r);

	bool rightside(Point p, LL segid);
	double getY(double l, Unit u);
	double getYpermin(LL segid, ProbUnit u, bool right);
	double query3Min(int year, int month, int day,
			LL segid, ProbUnit u, bool right);
	double queryWait(int year, int month, int day,
			LL segid, ProbUnit u, bool right);
	pair<double, double> query(string input);

	static int towardChinese(char *s);
	void test();
	void outputsegment();

	void save();
	void load();
};
#endif
