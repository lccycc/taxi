#ifndef ORIDATA_H
#define ORIDATA_H
#include "include/head.h"
#include "include/point.h"
#include "include/timestamp.h"

int getNormalWeekDay(int y, int M, int d);

class TaxiPoint{
public:
	char trigger;
	/*
	 * 0: turn empty
	 * 1: turn carry
	 * 2: set armor
	 * 3: unset armor
	 * 4 other
	 */
	char state;
	/*
	 * 0: empty
	 * 1: carry
	 * 2: stay somewhere
	 * 3: get off work
	 * 4: other
	 */
	LL time;
	Point pos;
	char v;
	short toward;

	int init(char *s);
	int getYear() const;
	int getHour() const;
	int getMinute() const;
	int getSecond() const;
	int getDay() const;
	int getWeekDay() const;
	int abstime() const;
	int timedis(TaxiPoint t) const;
	bool isempty() const;

	static Point gps2pnt(double gpsx, double gpsy);
};
bool cmpTaxiPointTime(const TaxiPoint &a, const TaxiPoint &b);

typedef vector<TaxiPoint> TaxiTrace;

class Taxi{
public:
	string name;

	string savepath;
	UMAP<int, LL> offset;
	ifstream tin;

	UMAP<int, TaxiTrace> tmptrace;

	Taxi();
	Taxi(string tname);
	void readoriginfile(char* filename);
	void readorigindata(string filename);
	void merge(vector<Taxi *> &taxis);
	void getTaxiTrace(int id, TaxiTrace &tr);
	void saveTaxiTrace(ofstream &fout, int id, TaxiTrace &tr);
	void savepiece();
	void loadTaxiTrace(ifstream &fin, TaxiTrace &tr);
	void load();
	//void savecache();
	//void loadcache();
	//void outputSpeedSegment(string outputfile);
};
#endif
