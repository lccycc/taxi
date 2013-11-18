#ifndef PROB_CPP
#define PROB_CPP
#include "prob.h"
Unit::Unit(){
	full = empt = 0;
	v = 0;
	memset(ts, 0, sizeof(ts));
}
/* XXX: if newtdis is too large, we should not valid tdis.
 * set an up bound for newtdis
 */
void Unit::overlap(double newv, int newtdis){
	v = (v*(full+empt) + newv)/(full+empt+1);
	tdis = (tdis*(full+empt) + newtdis)/(full+empt+1);
}

void Unit::addfull(double newv, int newtdis){
	overlap(newv, newtdis);
	full++;
}

void Unit::addempt(double newv, int newtdis){
	overlap(newv, newtdis);
	empt++;
}

ProbUnit::ProbUnit(){
	days = 0;
}

void ProbUnit::add(const TaxiPoint tp,
		bool ispvt, bool isempty, double newv, int newtdis, bool ifaddday){
	if (!ifaddday){
		if (ispvt && isempty){
			pvt.addempt(newv, newtdis);
		}else
		if (ispvt && !isempty){
			pvt.addfull(newv, newtdis);
		}else
		if (!ispvt && isempty){
			bac.addempt(newv, newtdis);
		}else{
			bac.addfull(newv, newtdis);
		}
	}else{
		if (isempty){
			int mt = tp.getMinute();
			int day = tp.getDay();
			int sec = tp.getSecond();
			if (mt > 59 || day > 30){
				cout<<"err "<<tp.time<<endl;
			}
			if (mt > 59) mt = 59;
			if (day > 30) day = 30;
			if (ispvt){
				pvt.ts[day][mt*6+sec/10]++;
			}else{
				bac.ts[day][mt*6+sec/10]++;
			}
		}
	}
}

ProbTestProfile::ProbTestProfile(){
	ypermin = -999;
	pos = Point(0, 0);
	weekday = 9;
	hour = 99;
	segid = 9999999;
	isright = false;
}

Prob::Prob(CrossCounter &x, string probname):cc(x){
	name = probname;
	setArgument();
	custommode = false;
}

void Prob::setArgument(){
	ifpvt_smalltoward = 40;
}

void Prob::countUnitDays(){
	for (int i = 1; i<=30; i++){
		for (int j = 0; j<24; j++){
			int timeid = getTimeID(getNormalWeekDay(2012, 11, i), j);
			for (auto ut = unitmap[timeid].begin();
					ut != unitmap[timeid].end(); ut++){
				ut->second.days++;
			}
		}
	}
}
bool Prob::ifpvt_must(Segment s, double toward){
	double stow = getSegTw(s);
	double mindis = angledis(stow, justtoward(toward, stow));
	if (fabs(justtoward(toward, stow)-toward) > 10){
		return 0;
	}else{
		return 1;
	}
}

/* ifpvt return:
 * 1  pvt
 * 0  bac
 * -1 invalid
 * never -1 here
 */
int Prob::ifpvt(Segment s, double toward){
	double stow = atan2(s.p2.y-s.p1.y, s.p2.x-s.p1.x)*180/M_PI;
	double mindis = angledis(stow, justtoward(toward, stow));
	if (mindis >= ifpvt_smalltoward){
		return -1;
	}
	if (fabs(justtoward(toward, stow)-toward) > 10){
		return 0;
	}else{
		return 1;
	}
}

int Prob::getWeekDayID(int weekday){
	switch (weekday){
	case 0:case 6: return 0;
	//case 6:	return 1;
	//case 5:	return 2;
	default:	return 1;
	}
}

/* return value at most  3*24 + 23 < 4*24 */
int Prob::getTimeID(int weekday, int hour){
	int rt = getWeekDayID(weekday)*24+hour;
	if (rt >= 4*24){
		cerr<<"error, weekday "<<weekday<<" hour "<<hour<<endl;
		assert(rt < 4*24);
	}
	return rt;
}

int Prob::getTimeID(const TaxiPoint tp){
	return getTimeID(tp.getWeekDay(), tp.getHour());
}

void Prob::add(const TaxiPoint tp, Segment s, int newtdis, bool ifaddday){
	int gettow = ifpvt(s, tp.toward);
	if (gettow == -1){
		return ;
	}
	add(tp, s, newtdis, gettow, ifaddday);
}

void Prob::add(const TaxiPoint tp, Segment s, int newtdis, bool ifpvt, bool ifaddday){
	int timeid = getTimeID(tp);
	unitmap[timeid][s.id].add(tp, ifpvt, tp.isempty(), tp.v, newtdis, ifaddday);
}

void Prob::add(const TaxiPoint tp, int newtdis, bool ifaddday){
	Point xps = tp.pos / cc.narrow;
	LL segid = cc.getSegmentID(xps);
	if (segid == -1){
		return ;
	}
	add(tp, cc.segment[segid], newtdis, ifaddday);
}

void Prob::overlap(const TaxiPoint tp, LL segid, int newtdis, bool ifaddday){
	add(tp, cc.segment[segid], newtdis, ifaddday);
}

USET<LL> Prob::footprint(const TaxiPoint taxipt, double second){
	vector<LL> visit = cc.footprint(taxipt, second);
	USET<LL> segids;
	USET<LL> us;
	for (int i = 0; i<visit.size(); i++){
		LL sgid = cc.getSegmentID(cc.getIPoint(visit[i]), 0);
		if (sgid == -1){
			continue;
		}
		segids.insert(sgid);
	}
	return segids;
}

vector<pair<TaxiPoint, TaxiPoint> > xl;

USET<LL> Prob::goPntPair(TaxiPoint tp, Point pto){
	double dd = dis(tp.pos, pto);
	double twd = getPntPairTw(tp.pos, pto);
	TaxiPoint fx = tp;
	fx.toward = twd;
	fx.v = 1;
	return footprint(fx, dd);
}

void Prob::goTaxiTrace(TaxiTrace &tr){
	USET<LL> last;
	LL lastid = -1;
	for (int i = 0; i+1 < tr.size(); i++){
		if (tr[i].getYear() < 2011 || tr[i+1].getYear()<2011){
			continue;
		}

		LL tsid = cc.getSegmentID(tr[i].pos / cc.narrow);
		if (tr[i].state == 0 && tsid != -1){
			overlap(tr[i], tsid, 0, true);
		}

		int newtdis = tr[i+1].timedis(tr[i]);
		double dd = dis(tr[i].pos, tr[i+1].pos);
		if (dd / cc.narrow < 500){
			if (cc.sameline(tr[i], tr[i+1], 10)){

				USET<LL> ths = goPntPair(tr[i], tr[i+1].pos);

				if (tr[i].state == 0 || tr[i].state == 1){
					for (auto jt = ths.begin();
							jt != ths.end(); jt++){
						if (last.find(*jt) == last.end()){
							overlap(tr[i], *jt, newtdis, false);
						}
					}
				}
				last = ths;
			}else{
				bool yy = false;
				Line l1(tr[i].pos, tr[i].pos + Point(tr[i].toward));
				Line l2(tr[i+1].pos, tr[i+1].pos + Point(tr[i+1].toward));
				Point s;
				USET<LL> ths, ths2;
				TaxiPoint ftr;
				if (intersect(l1, l2, s) && dis(tr[i].pos, s)<500 && dis(tr[i+1].pos, s)<500){
					yy =true;
					ths = goPntPair(tr[i], s);
					ftr = tr[i+1];
					ftr.pos = s;
					ths2 = goPntPair(ftr, tr[i+1].pos);
				}else{
					double tt = newtdis/2;
					ths = footprint(tr[i], tt);
					ftr = tr[i+1];
					ftr.pos = ftr.pos - tt*Point(ftr.toward);
					ths2 = goPntPair(ftr, tr[i+1].pos);
				}

				for (auto jt = ths.begin();
						jt != ths.end(); jt++){
					if (last.find(*jt) == last.end()){
						overlap(tr[i], *jt, newtdis, false);
					}
				}
				last = ths;
				for (auto jt = ths2.begin();
						jt != ths2.end(); jt++){
					if (last.find(*jt) == last.end()){
						overlap(ftr, *jt, newtdis, false);
					}
				}
				last = ths2;

				/*
				if (yy && dis(tr[i].pos, tr[i+1].pos) > 800 && (ths.size() +ths2.size()>5)){
					USET<LL> deth;
					deth.insert(ths.begin(), ths.end());
					deth.insert(ths2.begin(), ths2.end());
					ofstream fout("data/debug.out");
					Point p1 = tr[i].pos/3, p2 = tr[i+1].pos/3;
					fout<<p1.x<<' '<<p1.y<<' '<<p2.x<<' '<<p2.y<<endl;
					for (auto dt = deth.begin();
							dt != deth.end(); dt++){
						LL id = *dt;
						fout<<cc.segment[id].p1.x<<' '<<cc.segment[id].p1.y<<' '<<cc.segment[id].p2.x<<' '<<cc.segment[id].p2.y<<endl;
					}
					fout.close();
					system("./script/debug.py");
				}
				*/


			}
		}else{
			last.clear();
		}
	}
}

void Prob::goTaxiTrace(TaxiTrace &tr, RoadMap &r){
	USET<LL> last;
	LL lastid = -1;
	for (int i = 0; i+1 < tr.size(); i++){
		if (!(tr[i].state == 0 || tr[i].state == 1)){
			continue;
		}

		if (tr[i].getYear() < 2011 || tr[i+1].getYear()<2011){
			continue;
		}
		double dd = dis(tr[i].pos, tr[i+1].pos);
		//if (cc.sameline(tr[i], tr[i+1])){
		if (dd / cc.narrow < 300){
			USET<LL> ths;
			LL sid1 = cc.getSegmentIDUntil(tr[i].pos/cc.narrow, 1);
			LL sid2 = cc.getSegmentIDUntil(tr[i+1].pos/cc.narrow, 1);
			/*
			*/
			/*
			LL sid1 = cc.getSegmentIDForw(tr[i].pos/cc.narrow, tr[i].toward);
			LL sid2 = cc.getSegmentIDUntil(tr[i+1].pos/cc.narrow, -tr[i+1].toward);
			*/
			if (sid1 == -1 || sid2 == -1){
				last.clear();
				continue;
			}
			vector<pair<LL, bool> > res = r.fpa[sid1][sid2];

			for (int j = 0; j<res.size(); j++){
				int id = res[j].first;
				if (ths.find(id) != ths.end()) continue;
				ths.insert(id);
				if (last.find(id) != last.end()) continue;
				int newtdis = tr[i+1].timedis(tr[i]);

				add(tr[i], cc.segment[id], newtdis, res[j].second);
			}
			if (tr[i+1].pos.x > tr[i].pos.x + 50
				&& ((last.find(39311) != last.end()
					&& last.find(3832) == last.end()
					&& ths.find(3832) == ths.end()
					&& ths.find(12502) != ths.end())
				 ||(ths.find(39311) != ths.end()
					 &&ths.find(12502) != ths.end()
					 && ths.find(3832) == ths.end()))
			   ){
				cout<<tr[i-1].pos.x<<' '<<tr[i-1].pos.y<<' '<<tr[i].pos.x/3<<' '<<tr[i].pos.y/3
					<<' '<<tr[i+1].pos.x/3<<' '<<tr[i+1].pos.y/3<<' '
					<<ths.size()<<' ';
				for (auto kt = ths.begin();
						kt != ths.end(); kt++){
					cout<<*kt<<' ';
				}
				cout<<endl;
			}
			last = ths;
		}else{
			last.clear();
		}
	}
}

void Prob::goTaxi(Taxi &t, RoadMap &r){
	cout<<"\t\t\t\tProb::goTaxi begin,"<<TimeStamp::gettime();
	int cnt = 0;
	for (auto it = t.offset.begin();
			it != t.offset.end(); it++){
		TaxiTrace tr;
		t.getTaxiTrace(it->first, tr);
		//goTaxiTrace(tr, r);
		goTaxiTrace(tr);
		cerr<<++cnt<<TimeStamp::gettime();
		//if (cnt > 2000) break;
	}
	/*
	for (int i = 0; i<xl.size(); i++){
		cout<<xl[i].first.pos.x<<' '<<xl[i].first.pos.y
			<<' '<<xl[i].second.pos.x<<' '<<xl[i].second.pos.y<<endl;
	}
	*/
	cout<<"\t\t\t\tProb::goTaxi end,"<<TimeStamp::gettime();
}

bool Prob::rightside(Point p, LL segid){
	Segment s = cc.segment[segid];
	Point d1 = p - s.p1;
	Point d2 = s.p2 - s.p1;

	double res = d1.x*d2.y-d1.y*d2.x;
	return res>0;
}

double Prob::getY(double l, Unit u){
	double x1 = u.empt;
	double x2 = u.full;
	double x3 = u.empt * l / u.tdis / ((double)u.v/cc.narrow);
	cout<<x1<<' '<<x2<<' '<<x1+x2<<' '<<x3<<endl;
	return x1;
	//return x2;
}

double Prob::getYpermin(LL segid, ProbUnit u, bool right){
	double l = dis(cc.segment[segid].p1, cc.segment[segid].p2);
	//double ypermin = (double)(u.pvt.empt + u.bac.empt) / u.days;
	double ypermin;
	if (right){
		ypermin = getY(l, u.pvt);
	}else{
		ypermin = getY(l, u.bac);
	}
	//cout<<"udays: "<<u.days<<endl;
	ypermin /= u.days * 60;
	cout<<"Ypermin: "<<ypermin<<endl;
	return ypermin;
}

double Prob::query3Min(int year, int month, int day, LL segid, ProbUnit u, bool right){
	double ypermin = getYpermin(segid, u, right);
	ypermin *= 3;
	return ypermin / (1+ypermin);
	if (ypermin*3 > 1){
		return 1;
	}else{
		return ypermin*3;
	}
	/*
	*/
	Unit ru = right? u.pvt : u.bac;

	int cnt = 0;
	int tot = 0;
	for (int i = 1; i<=30; i++){
		if (getWeekDayID(getNormalWeekDay(2012, 11, i))
					== getWeekDayID(getNormalWeekDay(year, month, day))){
			for (int j = 0; j+3*6<=360; j++){
				bool hastaxi = false;
				for (int k = j; k<j+3*6; k++){
					hastaxi |= ru.ts[i][k];
				}
				cnt += hastaxi;
				tot++;
			}
		}
	}
	return (double)cnt/(double)tot;
}

double Prob::queryWait(int year, int month, int day, LL segid, ProbUnit u, bool right){
	/*
	double ypermin = getYpermin(segid, u, right);
	if (ypermin == 0){
		return 15;
	}
	return min(15.0, 1/ypermin);
	*/
	Unit ru = right? u.pvt : u.bac;

	int cnt = 0;
	int tot = 0;
	int len = 0;
	for (int i = 1; i<=30; i++){
		if (getWeekDayID(getNormalWeekDay(2012, 11, i))
					== getWeekDayID(getNormalWeekDay(year, month, day))){
			len = 0;
			for (int j = 0; j<360; j++){
				if (ru.ts[i][j]){
					len = 0;
				}else{
					len++;
				}
				cnt += len;
				tot++;
			}
		}
	}
	return (double)cnt/(double)tot/6;
}

int Prob::towardChinese(char *s){
	/*
	 * right 0
	 * down: 1
	 * left: 2
	 * up 3
	 */
	if (strcmp(s, "自西向东") == 0){
		return 0;
	}
	if (strcmp(s, "自北向南") == 0){
		return 1;
	}
	if (strcmp(s, "自东向西") == 0){
		return 2;
	}
	if (strcmp(s, "自南向北") == 0){
		return 1;
	}
	cerr<<"err "<<s<<endl;
	assert(false);
}

pair<double, double> Prob::query(string input){
	double gpsx, gpsy;
	int year, month, day, hour, minute, second;
	char zdxx[100];
	sscanf(input.c_str(), "(%lf, %lf)%s%d/%d/%d %d:%d:%d",
			&gpsx, &gpsy, zdxx, &year, &month, &day, &hour, &minute, &second);
	//cout<<"xxx"<<zdxx<<"xxx"<<endl;

	currentTest.hour = hour;
	Point p = TaxiPoint::gps2pnt(gpsx, gpsy)/cc.narrow;
	//cout<<"YYY "<<p.x<<' '<<p.y<<endl;
	currentTest.pos = p;

	int weekday = getNormalWeekDay(year, month, day);
	currentTest.weekday = weekday;
	cout<<"weekday "<<weekday<<" hour"<<hour<<endl;

	int timeid = getTimeID(weekday, hour);
	int inputtoward = towardChinese(zdxx);
	double times = TimeStamp::currentTime();
	LL sid = cc.getNearestSegmentID(p, inputtoward);
	if (sid <= 0){
		cout<<"no found!"<<endl;
		assert(0);
	}
	currentTest.segid = sid;
	currentTest.seg= cc.segment[sid];
	cout<<"seg: "<<sid<<' '<<cc.segment[sid].p1.x<<' '<<cc.segment[sid].p1.y<<' '<<cc.segment[sid].p2.x<<' '<<cc.segment[sid].p2.y<<endl;
	ProbUnit u = unitmap[timeid][sid];
	bool isrightmine = rightside(p, sid);
	bool isright = (inputtoward == CrossCounter::towardSeg(cc.segment[sid]));
	currentTest.isright = isright;
	currentTest.ypermin = getYpermin(sid, u, isright);


	double q3min = query3Min(year, month, day, sid, u, isright);
	double qwait = queryWait(year, month, day, sid, u, isright);

	testresult.push_back(currentTest);
	char timestr[30];
	sprintf(timestr, "%.6f", (TimeStamp::currentTime() - times)/1000000);
	cerr<<q3min<<' '<<qwait<<' '<<timestr<<endl;
	return make_pair(q3min, qwait);
}

void Prob::test(){
	testresult.clear();
	string testfile = "data/prob/final.txt";
	//string testfile = "data/prob/test.txt";
	//string testfile = "data/prob/guomao_5_17_mine.txt";
	//string testfile = "data/prob/tiantan.txt";
	string tmp;
	ifstream fin(testfile);
	getline(fin, tmp);
	int id = 0;
	Point lastpt(0, 0);
	while (getline(fin, tmp)){
		if (tmp[tmp.size()-1] == 13){
			tmp = tmp.substr(0, tmp.size()-1);
		}
		cout<<id<<"------"<<tmp<<"-------"<<endl;
		id++;
		currentTest = ProbTestProfile();
		pair<double, double> res = query(tmp);
		cout<<res.first<<' '<<res.second<<endl;
	}
	fin.close();
	outputsegment();
}

void Prob::outputsegment(){
	string filename = "probout";
	string probout= "data/prob/"+filename+".dat";
	cc.outputsegment(false, filename);
	ofstream fout(probout.c_str());
	int pointid = 0;
	for (int i = 0; i<testresult.size(); i++){
		ProbTestProfile ti = testresult[i];
		if (i>0 && dis(testresult[i-1].pos, testresult[i].pos)>2){
			pointid++;
		}

		fout<<pointid<<' '<<ti.weekday<<' '<<ti.hour<<' '<<ti.ypermin<<' '
			<<ti.pos.x<<' '<<ti.pos.y<<' '
			<<ti.segid<<' '
			<<ti.seg.p1.x<<' '<<ti.seg.p1.y<<' '
			<<ti.seg.p2.x<<' '<<ti.seg.p2.y<<' '
			<<ti.isright<<endl;
	}
	fout.close();
}

void Prob::save(){
	string probfile = "data/prob/" + name + ".dat";
	ofstream fout(probfile.c_str(), ios::out|ios::binary);
	for (int i = 0; i<4*24; i++){
		int utsize = unitmap[i].size();
		cout<<utsize<<endl;
		fout.write((char*)&utsize, sizeof(utsize));
		for (auto jt = unitmap[i].begin();
				jt != unitmap[i].end(); jt++){
			pair<LL, ProbUnit> tmp = *jt;
			fout.write((char*)&tmp, sizeof(tmp));
			fout.flush();
		}
	}
	cout<<"finish"<<endl;
	//fout.close();
	cout<<"\t\t\t\tProb::save end,"<<TimeStamp::gettime();
}

void Prob::load(){
	string probfile = "data/prob/" + name + ".dat";
	ifstream fin(probfile.c_str(), ios::in|ios::binary);
	for (int i = 0; i<4*24; i++){
		int utsize;
		fin.read((char*)&utsize, sizeof(utsize));
		unitmap[i].clear();
		for (int j = 0; j<utsize; j++){
			pair<LL, ProbUnit> punit;
			fin.read((char*)&punit, sizeof(punit));
			unitmap[i].insert(punit);
		}
	}
	fin.close();
	countUnitDays();
	cout<<"\t\t\t\tProb::load end,"<<TimeStamp::gettime();
}

void Prob::buildcustom(){
	for (auto it = cc.segment.begin();
			it != cc.segment.end(); it++){
		for (int i = 1; i<=7; i++){
			for (int j = 0; j<24; j++){
				int weekday = getNormalWeekDay(2012, 11, i);
				int timeid = getTimeID(weekday, j);
				double l1 = query3Min(2012, 11, i, it->first, unitmap[timeid][it->first], true);
				double r1 = queryWait(2012, 11, i, it->first, unitmap[timeid][it->first], true);
				double l2 = query3Min(2012, 11, i, it->first, unitmap[timeid][it->first], false);
				double r2 = queryWait(2012, 11, i, it->first, unitmap[timeid][it->first], false);

				cpvt[timeid][it->first] = make_pair(l1, r1);
				cbac[timeid][it->first] = make_pair(l2, r2);
			}
		}
	}
	string probfile = "data/prob/" + name + "_custom.dat";
	ofstream fout(probfile.c_str(), ios::out|ios::binary);
	for (int i = 0; i<4*24; i++){
		int utsize = cpvt[i].size();
		fout.write((char*)&utsize, sizeof(utsize));
		for (auto jt = cpvt[i].begin();
				jt != cpvt[i].end(); jt++){
			fout.write((char*)&(*jt), sizeof(*jt));
		}
		for (auto jt = cbac[i].begin();
				jt != cbac[i].end(); jt++){
			fout.write((char*)&(*jt), sizeof(*jt));
		}
	}
	fout.close();
	cout<<"BuildCustom end"<<endl;
}
void Prob::loadcustom(){
	string probfile = "data/prob/" + name + "_custom.dat";
	ifstream fin(probfile.c_str(), ios::in|ios::binary);
	assert(fin);
	for (int i = 0; i<4*24; i++){
		int utsize;
		fin.read((char*)&utsize, sizeof(utsize));
		for (int j = 0; j<utsize; j++){
			pair<LL, pair<double, double> > d;
			fin.read((char*)&d, sizeof(d));
			cpvt[i].insert(d);
		}
		for (int j = 0; j<utsize; j++){
			pair<LL, pair<double, double> > d;
			fin.read((char*)&d, sizeof(d));
			cbac[i].insert(d);
		}
	}
	fin.close();
	custommode = true;
	cout<<"Prob::customload end"<<endl;
}
void Prob::term(){
	string tmp;
	while (getline(cin, tmp)){
		if (tmp[0] == 'E') break;
		if (tmp[tmp.size()-1] == 13){
			tmp = tmp.substr(0, tmp.size()-1);
		}
		double gpsx, gpsy;
		int year, month, day, hour, minute, second;
		char zdxx[100];
		sscanf(tmp.c_str(), "(%lf, %lf)%s%d/%d/%d %d:%d:%d",
			&gpsx, &gpsy, zdxx, &year, &month, &day, &hour, &minute, &second);
		Point p = TaxiPoint::gps2pnt(gpsx, gpsy)/cc.narrow;
		int weekday = getNormalWeekDay(year, month, day);
		int timeid = getTimeID(weekday, hour);
		int inputtoward = towardChinese(zdxx);
		double times = TimeStamp::currentTime();
		LL sid = cc.getNearestSegmentID(p, inputtoward);
		bool isright = (inputtoward == CrossCounter::towardSeg(cc.segment[sid]));
		char timestr[30];
		sprintf(timestr, "%.6f", (TimeStamp::currentTime() - times)/1000000);
		double l, r;
		if (isright){
			l = cpvt[timeid][sid].first;
			r = cpvt[timeid][sid].second;
		}else{
			l = cbac[timeid][sid].first;
			r = cbac[timeid][sid].second;
		}
		cout<<tmp<<'\t'<<l<<'\t'<<r<<'\t'<<timestr<<endl;
	}
}
#endif

