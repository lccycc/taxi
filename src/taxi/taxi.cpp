#ifndef ORIDATA_CPP
#define ORIDATA_CPP
#include "taxi.h"
int getNormalWeekDay(int y, int M, int d){
	int c= y/100;
	y = y % 100;
	if (M <= 2){
		M += 12;
	}
	int w = c/4 - 2*c + y + y/4 + (13*(M+1)/5) + d - 1;

	return ((w % 7)+7)%7;
}

int TaxiPoint::getYear() const{
	return (time/1000000LL)/10000;
}
int TaxiPoint::getHour() const{
	return time/10000 % 100;
}
int TaxiPoint::getMinute() const{
	return time/100 % 100;
}
int TaxiPoint::getSecond() const{
	return time%100;
}
int TaxiPoint::getDay() const{
	return time/1000000 % 100;
}
int TaxiPoint::getWeekDay() const{
	int d = getDay();
	/* 2012.11.1 is Thursday */
	return (d+3)%7;
}
int TaxiPoint::abstime() const{
	int d = getDay();
	int h = getHour();
	int m = getMinute();
	int s = getSecond();
	return (((d-1)*24 + h)*60+m)*60+s;
}
int TaxiPoint::timedis(TaxiPoint t) const{
	return abstime() - t.abstime();
}
bool TaxiPoint::isempty() const{
	return state == 0;
}

Point TaxiPoint::gps2pnt(double gpsx, double gpsy){
	const static double rate = M_PI/180.0;
	const static double hx = 116.397 * rate, hy = 39.914*rate;

	const static double R = 6731000;
	const static double r = R * cos(hy);
	double dx = (gpsx*rate-hx) * r;
	double dy = (gpsy*rate-hy) * R;
	return Point(dx, dy);
}

int TaxiPoint::init(char *s){
	int id = -1;
	bool valid = false;
	double x, y;
	int in_tow, in_tri, in_stat;
	double in_v;
	sscanf(s, "%d,%d,%d,%lld,%lf,%lf,%lf,%d,%d",
			&id, &in_tri, &in_stat, &time, &x, &y, &in_v, &in_tow, &valid);
	trigger = in_tri;
	state = in_stat;

	v = in_v/3.6;
	in_tow = ((360-in_tow)-90 + 360)%360;
	//toward = (in_tow+22)%360 / 45;
	toward = in_tow;

	pos = gps2pnt(x, y);
	//if (fabs(pos.x) > 30000 || fabs(pos.y) > 30000){
	//if (fabs(pos.x) > 5000 || fabs(pos.y) > 5000){
	if (fabs(pos.x) > 10000 || fabs(pos.y) > 10000){
	//if (pos.x/3 < -100 || pos.x/3 > 3000 || pos.y/3 < -2500 || pos.y/3 > 600){
	//if (pos.x/3 < -1100 || pos.x/3 > 4000 || pos.y/3 < -3500 || pos.y/3 > 1600){
		valid = false;
	}

	return valid?id:-1;
}
bool cmpTaxiPointTime(const TaxiPoint &a, const TaxiPoint &b){
	return a.abstime()< b.abstime();
}

Taxi::Taxi(string tname){
	name = tname;
	savepath = "data/taxi/"+name+".dat";
}

void Taxi::readoriginfile(char* filename){
	char tmp[100];
	FILE *fin = fopen(filename, "r");
	while (fscanf(fin, "%s", tmp)!=EOF){
		TaxiPoint tp;
		int id = tp.init(tmp);
		if (id>=0){
			tmptrace[id].push_back(tp);
		}
	}
	fclose(fin);
}
void Taxi::readorigindata(string filelist){
	FILE *flist = fopen(filelist.c_str(), "r");
	assert(flist);
	char filename[200];
	vector<Taxi *> pieces;
	int filecnt = 0;
	while (fscanf(flist, "%s", filename)!=EOF){
		if (filecnt++ % (24*60) == 0){
			if (pieces.size()){
				pieces.back()->savepiece();
			}
			char news[20];
			sprintf(news, "piece/%02d", pieces.size());
			pieces.emplace_back(new Taxi(news));
		}
		cout<<filename<<endl;
		pieces.back()->readoriginfile(filename);
	}
	pieces.back()->savepiece();

	merge(pieces);
	for (int i = 0; i<pieces.size(); i++){
		delete pieces[i];
	}
	cout<<"\t\t\tread origin data finish,"<<TimeStamp::gettime();
	cout<<"taxi Number: "<<offset.size()<<endl;

	tin.open(savepath.c_str(), ios::in|ios::binary);
}

void Taxi::merge(vector<Taxi *> &taxis){
	offset.clear();
	for (int i = 0; i<taxis.size(); i++){
		offset.insert(taxis[i]->offset.begin(), taxis[i]->offset.end());
	}

	ofstream fout(savepath.c_str(), ios::out|ios::binary);
	int idcnt = offset.size();
	fout.write((char*)&idcnt, sizeof(idcnt));
	for (auto ut = offset.begin(); ut != offset.end(); ut++){
		fout.write((char*)&(*ut), sizeof(*ut));
	}

	for (auto ut = offset.begin(); ut != offset.end(); ut++){
		int id = ut->first;
		TaxiTrace tr, tmp;
		for (int i = 0; i<taxis.size(); i++){
			taxis[i]->getTaxiTrace(id, tmp);
			tr.insert(tr.end(), tmp.begin(), tmp.end());
		}
		sort(tr.begin(), tr.end(), cmpTaxiPointTime);
		offset[id] = fout.tellp();
		saveTaxiTrace(fout, id, tr);
	}

	fout.seekp(sizeof(idcnt), ios::beg);
	for (auto ut = offset.begin(); ut != offset.end(); ut++){
		fout.write((char*)&(*ut), sizeof(*ut));
	}
	fout.close();
	cout<<"Taxi::merge end"<<TimeStamp::gettime();
}

void Taxi::getTaxiTrace(int id, TaxiTrace &tr){
	tr.clear();
	if (offset.find(id) == offset.end()){
		return ;
	}
	tin.seekg(offset[id], ios_base::beg);
	loadTaxiTrace(tin, tr);
}

void Taxi::saveTaxiTrace(ofstream &fout, int id, TaxiTrace &tr){
	int sz = tr.size();
	fout.write((char*)&id, sizeof(id));
	fout.write((char*)&sz, sizeof(sz));
	fout.write((char*)&tr[0], sizeof(tr[0])*sz);
}

void Taxi::savepiece(){
	ofstream fout(savepath.c_str(), ios::out|ios::binary);
	for (auto it = tmptrace.begin();
			it != tmptrace.end(); it++){
		offset[it->first] = fout.tellp();
		saveTaxiTrace(fout, it->first, it->second);
	}
	tmptrace.clear();
	fout.close();
	cout<<"\tTaxi::savepiece "<<savepath<<TimeStamp::gettime();

	tin.open(savepath.c_str(), ios::in|ios::binary);
}
void Taxi::loadTaxiTrace(ifstream &fin, TaxiTrace &tr){
	int id, sz;
	fin.read((char*)&id, sizeof(id));
	fin.read((char*)&sz, sizeof(sz));
	tr.resize(sz);
	fin.read((char*)&tr[0], sz*sizeof(tr[0]));
	/* XXX for debug */
	/*
	for (int i = tr.size()-1; i>=0; i--){
		if (fabs(tr[i].pos.x)>5000 || fabs(tr[i].pos.y)>5000){
			tr.erase(tr.begin()+i);
		}
	}
	*/
}
void Taxi::load(){
	tin.open(savepath.c_str(), ios::in|ios::binary);
	assert(tin);

	int taxinum;
	tin.read((char*)&taxinum, sizeof(taxinum));
	cout<<"taxi ID num: "<<taxinum<<endl;

	offset.clear();
	for (int i = 0;i<taxinum; i++){
		pair<int, LL> off;
		tin.read((char*)&off, sizeof(off));
		offset.insert(off);
	}
	TaxiTrace tmp;

	/*
	int tot = 0;
	for (auto it = offset.begin();
			it != offset.end(); it++){
		getTaxiTrace(it->first, tmp);
		tot += tmp.size();
	}
	cout<<"load end, taxi num = "<<tot<<endl;
	*/
}
/*
void Taxi::savecache(){
	const static string taxisavepnt = "taxisavepnt";
	const static string taxisaveids = "taxisaveids";
	vector<int> ids;
	for (auto it = trace.begin();
			it!=trace.end(); it++){
		ids.push_back(it->first);
	}
	MemCache::savecache(ids, taxisaveids, -1);
	for (auto it = trace.begin();
			it!=trace.end(); it++){
		MemCache::savecache(it->second, taxisavepnt, it->first);
	}
	cout<<"\t\t\ttaxi savecache finish,"<<TimeStamp::gettime();
}
void Taxi::loadcache(){
	const static string taxisavepnt = "taxisavepnt";
	const static string taxisaveids = "taxisaveids";
	vector<int> ids;
	MemCache::loadcache(ids, taxisaveids, -1);
	trace.clear();
	LL totalpoint = 0;
	for (int i = 0; i<ids.size(); i++){
		MemCache::loadcache(trace[ids[i]], taxisavepnt, ids[i]);
		totalpoint += trace[ids[i]].size();
	}
	cout<<"Taxi::loadcache taxinum = "<<trace.size()<<" totalpoint = "<<totalpoint<<endl;
	cout<<"\t\t\ttaxi loadcache finish,"<<TimeStamp::gettime();
}
*/
/*
void Taxi::outputSpeedSegment(string outputfile){
	int sec = 5;
	ofstream fout(outputfile.c_str());
	for (auto it = trace.begin();
			it != trace.end(); it++){
		for (int i = 0; i<it->second.size(); i++){
			TaxiPoint &pt = it->second[i];
			 if (pt.pos.x<4000 || pt.pos.x>4400 || pt.pos.y<4100 || pt.pos.y> 4500) continue;
			 if (pt.toward > 35 && pt.toward < 55 || pt.toward > 215 && pt.toward < 235) continue;
			Point a = pt.pos;
			Point b = a + sec * pt.v * Point(pt.toward);
			fout<<(int)a.x<<' '<<(int)a.y<<' '<<(int)b.x<<' '<<(int)b.y<<endl;
		}
	}
	fout.close();
}
*/

#endif
