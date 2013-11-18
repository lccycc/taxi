#ifndef CROSS_CPP
#define CROSS_CPP
#include "cross.h"

double getSegTw(Segment s){
	return angleform(atan2(s.p2.y - s.p1.y, s.p2.x-s.p1.x)*180/M_PI);
}
double getPntPairTw(Point p1, Point p2){
	return angleform(atan2(p2.y - p1.y, p2.x-p1.x)*180/M_PI);
}

double linear(vector<IPoint> ip, double toward, double &width){
	vector<Point> p;
	for (int i = 0; i<ip.size(); i++){
		p.push_back(rotate(ip[i], -toward));
	}

	double A=0.0,B=0.0,C=0.0,D=0.0,E = 0;
	int n = ip.size();
	for(int i=0;i<n;i++){
		A+=p[i].x;
		B+=p[i].y;
		C+=p[i].x*p[i].x;
		D+=p[i].x*p[i].y;
		E+=p[i].y*p[i].y;
	}
	double m1 =(n*D-A*B)/(n*C-A*A);
	double m2 = (n*D-A*B)/(n*E-B*B);
	double c1 = (B-m1*A)/n;
	double c2 = (A-m2*B)/n;
	double deta1 =0, deta2 = 0;
	for (int i = 0; i<n; i++){
		double d1 = fabs(m1*p[i].x+c1-p[i].y);
		double d2 = fabs(m2*p[i].y+c2-p[i].x);
		deta1 += d1*d1;
		deta2 += d2*d2;
	}
	double maxd;
	Line l(m1, -1, c1);
	maxd = 0;
	for (int i = 0; i<n; i++){
		double d = fabs(l.f(p[i]));
		maxd += d;
	}
	maxd /= n;
	width = maxd*2;
	double rs = atan2(-l.a, l.b);
	rs = rs/M_PI*180;
	rs = angleform(rs);
	rs = angleform(rs + toward);
	return rs;
}
Cross::Cross(){
	toward = 0;
	counter = 0;
}

void Cross::overlap(double overtow){
	overtow = justtoward(overtow, toward);
	toward = (toward*counter + overtow)/(counter+1);
	counter++;
}
void Cross::overlap(double overtow, int weight){
	overtow = justtoward(overtow, toward);
	toward = (toward*counter + overtow*weight)/(counter+weight);
	counter += weight;
}

Segment::Segment(){
	width = 0;
	valid = true;
}
Segment::Segment(Point _p1, Point _p2){
	p1 = _p1, p2 = _p2;
	width = 0;
	valid = true;
}

Line::Line(){
	a = b = c = 0;
}
Line::Line(double _a, double _b, double _c){
	double one = sqrt(sqr(_a)+sqr(_b));
	a = _a/one;
	b = _b/one;
	c = _c/one;
}
Line::Line(Point p, Point q){
	a = q.y - p.y;
	b = p.x - q.x;
	c = p.y*q.x - p.x*q.y;
	double one = sqrt(sqr(a) + sqr(b));
	a/=one; b/=one; c/=one;
}
double Line::f(Point p){
	return a*p.x + b*p.y + c;
}
double Line::toward(){
	return atan2(-b, a)*180/M_PI;
}
bool intersect(Line p, Line q, Point &s){
	if (fabs(p.a*q.b - p.b*q.a) < 1e-6) return false;
	s.y=(p.a*q.c-p.c*q.a)/(q.a*p.b-p.a*q.b);
	s.x=(p.b*q.c-p.c*q.b)/(p.a*q.b-p.b*q.a);
	return true;
}

CrossCounter::CrossCounter(string _name):name(_name){
	setArgument();
}
LL CrossCounter::getID(int x, int y){
	return (x+baseX/2)*baseX + y+baseX/2;
}
Cross& CrossCounter::getCross(LL id){
	return ccounter[id];
}
Cross& CrossCounter::getCross(int x, int y){
	return ccounter[getID(x, y)];
}
Cross& CrossCounter::getCross(IPoint p){
	return ccounter[getID(p.x, p.y)];
}
bool CrossCounter::exist(int x, int y){
	return ccounter.find(getID(x, y)) != ccounter.end();
}
IPoint CrossCounter::getIPoint(LL id){
	return IPoint(id/baseX-baseX/2, id%baseX-baseX/2);
}

bool CrossCounter::sameline(TaxiPoint t1, TaxiPoint t2){
	return sameline(t1, t2, towardfilter);
}

bool CrossCounter::sameline(TaxiPoint t1, TaxiPoint t2, double towardfilter){
	double gotime = t2.abstime()-t1.abstime();
	if (gotime < 2 || gotime > 300) return false;
	double godis = t1.v * gotime;
	if (dis(t1.pos, t2.pos) < 3.0) return false;
	//if (dis(t1.pos, t2.pos) > godis*2) return false;
	Point dta = t2.pos - t1.pos;
	double realtw = atan2(dta.y, dta.x)/M_PI*180;
	realtw = justtoward(realtw, t1.toward);
	if (angledis(realtw, t1.toward)> towardfilter){
		return false;
	}
	realtw = justtoward(realtw, t2.toward);
	if (angledis(realtw, t2.toward)> towardfilter){
		return false;
	}

	return true;
}

vector<LL> CrossCounter::footprint(TaxiPoint taxipt, double second){
	taxipt.pos = taxipt.pos / narrow;
	Point mov = taxipt.pos;
	Point ptow(taxipt.toward);
	int x, y, lastx = -999999999, lasty = -99999999;
	USET<LL> visit;
	vector<LL> res;
	for (int i = 0; i<=ceil(taxipt.v*second/narrow); i++){
		mov = taxipt.pos + i * ptow;
		x = mov.x, y = mov.y;
		if (x != lastx || y != lasty){
			for (int dx = -overlapWidth; dx<=overlapWidth; dx++){
				for (int dy = -overlapWidth; dy<=overlapWidth; dy++){
					LL ID = getID(x+dx, y+dy);
					if (visit.find(ID)!=visit.end()) continue;
					visit.insert(ID);
					res.push_back(ID);
				}
			}
		}
		lastx = x, lasty = y;
	}
	return res;
}

void CrossCounter::overlap(TaxiPoint taxipt, double second,
							CrossCounter *notcover){
	double overtow = taxipt.toward;
	/*
	if (regionLimit && (
		taxipt.pos.x<regionLimit1.x || taxipt.pos.y<regionLimit1.y ||
		taxipt.pos.x>regionLimit2.x || taxipt.pos.y>regionLimit2.y)){
			return ;
	}
	*/
	vector<LL> visit = footprint(taxipt, second);
	for (auto it = visit.begin();
			it != visit.end(); it++){
		/*if (notcover != NULL && notcover->getSegmentID(getIPoint(*it)) >=0 ){
			continue;
		}*/
		getCross(*it).overlap(overtow);
	}
}
void CrossCounter::setArgument(){
	narrow = 3;

	regionLimit = false;
	/*
	 * final data
	regionLimit1 = Point(-100, -2500);
	regionLimit2 = Point(3000, 600);
	*/

	towardfilter = 60;
	speedfilter = 10;
	smallrunsecond = 1;

	gotaxiNightlimit = false;
	avertowRange = 20 / narrow;
	avertowEnable = true;
	bfspointRange = 10;
	bfspointHeartdis = 100 / narrow;
	bfspointCounterlb = 500;
	getsegmentMidlenlb = 10 / narrow;
	getsegmentMaxWidth= 99999 / narrow;
	overlapWidth = 1;
}
void CrossCounter::goTaxi(Taxi &t, CrossCounter *c){
	TimeStamp::gettime();
	ccounter.clear();
	for (auto it = t.offset.begin();
				it!=t.offset.end(); it++){
		TaxiTrace tr;
		t.getTaxiTrace(it->first, tr);
		goTaxiTrace(tr, c);
	}
	cout<<"ccounter.size = "<<ccounter.size()<<endl;
	cout<<"\t\tCrossCounter::goTaxi finish,"<<TimeStamp::gettime();
}
void CrossCounter::goTaxiTrace(TaxiTrace &tr, CrossCounter *c){
	if (!c){
	//if (false){
		for (auto jt = tr.begin();
				jt != tr.end(); jt++){
			TaxiPoint &taxipt = *jt;
			if (gotaxiNightlimit &&
					taxipt.getHour()>7 && taxipt.getHour() <= 22){
				continue;
			}
			Point ip = taxipt.pos / narrow;
			if (regionLimit && (
					ip.x < regionLimit1.x || ip.y<regionLimit1.y
					|| ip.x > regionLimit2.x || ip.y > regionLimit2.y)){
				continue;
			}
			if (taxipt.v < speedfilter){
				continue;
			}
			overlap(taxipt, smallrunsecond, c);
		}
	}else{
		for (int i = 1; i<tr.size(); i++){
			Point ip = tr[i-1].pos / narrow;
			if (regionLimit && (
					ip.x < regionLimit1.x || ip.y<regionLimit1.y
					|| ip.x > regionLimit2.x || ip.y > regionLimit2.y)){
				continue;
			}
			if (sameline(tr[i-1], tr[i])){
				tr[i-1].toward = atan2(tr[i].pos.y - tr[i-1].pos.y,
									tr[i].pos.x - tr[i-1].pos.x)/M_PI*180;
				double tt = dis(tr[i-1].pos, tr[i].pos)/tr[i-1].v;
				//tt*=2;
				tt += smallrunsecond;
				overlap(tr[i-1], tt, c);
			}
		}
	}
}
void CrossCounter::countAvertow(){
	//cout<<"\t\tCrossCounter::countAvertow begin,"<<TimeStamp::gettime();
	for (auto it = ccounter.begin();
			it != ccounter.end(); it++){
		IPoint h = getIPoint(it->first);
		/*no avertow*/
		if (!avertowEnable){
			it->second.avertow = it->second.toward;
			continue;
		}

		int avertow = 0;
		int avercnt = 0;
		Cross cs;
		for (int dx = -avertowRange; dx<=avertowRange; dx++){
			for (int dy = -avertowRange; dy<=avertowRange; dy++){
				if (!exist(h.x+dx, h.y+dy)) continue;
				Cross &c = getCross(h.x+dx, h.y+dy);
				int tow = justtoward(c.toward, avertow);
				cs.overlap(tow, c.counter);
			}
		}
		it->second.avertow = cs.toward;
	}
	//cout<<"\t\tCrossCounter::countAvertow finish,"<<TimeStamp::gettime();
}

void CrossCounter::outputpoint(){
	const string ccoutputpoint = "data/road/ccpoint.dat";
	ofstream fout(ccoutputpoint.c_str());
	LL totpnt = 0;
	for (auto it = ccounter.begin();
			it != ccounter.end(); it++){
		if (it->second.counter > 20){
			IPoint ip = getIPoint(it->first);
			Point p = Point(ip.x, ip.y) + Point(it->second.toward)*15;
			fout<<ip.x<<' '<<ip.y<<' '<<p.x<<' '<<p.y<<endl;
			totpnt++;
		}
	}
	fout.close();
	cout<<"CrossCounter::outputpoint totpnt = "<<totpnt<<endl;
	cout<<"\tCrossCounter::outputpoint finish,"<<TimeStamp::gettime();
}

vector<IPoint> CrossCounter::bfspoint(IPoint start, double &toward,
									UMAP<LL, LL> &visit, LL segid){
	vector<IPoint> res;
	visit[getID(start.x, start.y)] = segid;
	res.push_back(start);
	Cross tmpcx;
	tmpcx.toward = toward;
	Point pheart(start);
	for (int i = 0; i<res.size(); i++){
		IPoint h = res[i];
		for (int dx = -bfspointRange; dx<=bfspointRange;dx++){
			for (int dy = -bfspointRange; dy<=bfspointRange; dy++){
				int x = h.x + dx, y = h.y + dy;
				if (dis(Point(x, y), pheart)>bfspointHeartdis) continue;
				LL ID = getID(x, y);
				if (!exist(x, y)) continue;
				Cross &c = getCross(ID);
				if (c.counter < bfspointCounterlb) continue;
				double xtoward = justtoward(c.avertow, tmpcx.toward);
				if (angledis(xtoward, tmpcx.toward)>towardfilter){
					continue;
				}
				if (visit.find(ID) != visit.end()){
					continue;
				}
				pheart = (pheart * res.size() + Point(x, y))/(res.size()+1);
				visit[ID] = segid;
				gridsegxid[getID(x/10, y/10)].insert(segid);
				tmpcx.overlap(xtoward);
				res.push_back(IPoint(x, y));
			}
		}
	}
	toward = tmpcx.toward;
	return res;
}
void CrossCounter::bfs(){
	countAvertow();
	cout<<"CrossCounter::bfs begin,"<<TimeStamp::gettime();
	segment.clear();
	xvisit.clear();
	gridsegxid.clear();
	LL xsegid = 0;
	for (auto it = ccounter.begin();
			it != ccounter.end(); it++){
		if (xvisit.find(it->first) != xvisit.end()){
			continue;
		}
		IPoint ip = getIPoint(it->first);
		/*
		if (regionLimit && (
				ip.x < regionLimit1.x || ip.y<regionLimit1.y
				|| ip.x > regionLimit2.x || ip.y > regionLimit2.y)){
			continue;
		}
		*/
		//warning: here toward will be changed during bfspoint
		double toward = it->second.toward;
		xsegid++;
		assert(xsegid < 2100000000);
		vector<IPoint> res = bfspoint(ip, toward, xvisit, xsegid);
		getSegmentFromPoints(res, xsegid);
	}
	cout<<"\t\tCrossCounter::bfs finish,"<<TimeStamp::gettime();
	fishEat(segment);
}
void CrossCounter::getSegmentFromPoints2(vector<IPoint> &ip, LL segid){
	Point averpmid;
	Cross ccss;
	for (int i = 0; i<ip.size(); i++){
		ccss.overlap(getCross(ip[i]).avertow);
		averpmid.x += ip[i].x;
		averpmid.y += ip[i].y;
	}
	averpmid = averpmid / ip.size();
	Point pmid = averpmid;

	Line bestline;
	double besti;
	double mindis = -1;
	double mmdds1, mmdds2;
	for (int i = ccss.toward-10; i<ccss.toward+10; i++){
		Point tow(i);
		Line l(tow.y, -tow.x, -(tow.y*pmid.x-tow.x*pmid.y));
		double tot = 0;
		double dds1 = 0, dds2 = 0;
		int cdds1 = 0, cdds2 = 0;
		for (int j = 0; j<ip.size(); j++){
			tot += fabs(l.f(Point(ip[j])));
			//tot = max(tot, fabs(l.f(Point(ip[j]))));
			double dds = tow.x*(ip[j].x-pmid.x) + tow.y*(ip[j].y-pmid.y);
			if (dds>0){
				dds1 = dds1+dds;
				cdds1++;
				//dds1 = max(dds, dds1);
			}else{
				dds2 = dds2+dds;
				cdds2++;
				//dds2 = min(dds, dds2);
			}
		}
		if (cdds1){
			dds1 = dds1/cdds1*2;
		}
		if (cdds2){
			dds2 = dds2/cdds2*2;
		}
		if (mindis<0 || mindis > tot){
			mindis = tot;
			mmdds1 = dds1;
			mmdds2 = dds2;
			bestline = l;
			besti = i;
		}
	}
	double midlength = 0;
	for (int i = 0; i<ip.size(); i++){
		//midlength = max(midlength, dis(pmid, ip[i]));
		midlength += dis(pmid, ip[i]);
	}
	midlength = midlength / ip.size()*2;

	midlength = (mmdds1-mmdds2)/2;


	double midwidth = 0;
	for (int i = 0; i<ip.size(); i++){
		midwidth += fabs(bestline.f(Point(ip[i])));
	}
	midwidth = midwidth / ip.size() * 2;
	double toward = besti;

	//Point p1 = pmid + midlength*Point(toward);
	//Point p2 = pmid - midlength*Point(toward);
	Point p1 = pmid + mmdds1*Point(toward);
	Point p2 = pmid + mmdds2*Point(toward);
	if (midlength>getsegmentMidlenlb){
		Segment seg(p1, p2);
		seg.id = segid;
		seg.width = midwidth*2;
		segment[segid] = seg;
	}
}



void CrossCounter::getSegmentFromPoints(vector<IPoint> &ip, LL segid){
	IPoint maxp = ip[0], minp = ip[0];
	Cross ccss;
	for (int i = 0; i<ip.size(); i++){
		maxp = maxp<ip[i]?ip[i]:maxp;
		minp = ip[i]<minp?ip[i]:minp;
	}
	//sort(ip.begin(), ip.end());
	Point pmid;
	IPoint ipmid;
	Point averpmid;
	for (int i = 0; i<ip.size(); i++){
		ccss.overlap(getCross(ip[i]).avertow);
		averpmid.x += ip[i].x;
		averpmid.y += ip[i].y;
	}
	averpmid = averpmid / ip.size();
	pmid = averpmid;

	double midlength;

	midlength = 0;
	for (int i = ip.size(); i>=0; i--){
		double d = dis(pmid, ip[i]);
		midlength = max(midlength, d);
	}

	double midwidth;
	double toward = linear(ip, ccss.toward, midwidth);

	double dds1 = 0, dds2 = 0;
	int cdds1 = 0, cdds2 = 0;
	Point tow(toward);

	for (int i = 0; i<ip.size(); i++){
		double dds = tow.x*(ip[i].x-pmid.x) + tow.y*(ip[i].y-pmid.y);
		if (dds>0){
			dds1 = dds1+dds;
			cdds1++;
			//dds1 = max(dds, dds1);
		}else{
			dds2 = dds2+dds;
			cdds2++;
			//dds2 = min(dds, dds2);
		}
	}
	if (cdds1){
		dds1 = dds1/cdds1*2;
	}
	if (cdds2){
		dds2 = dds2/cdds2*2;
	}

	/*
	Point p1 = pmid + midlength*tow;
	Point p2 = pmid - midlength*tow;
	*/
	Point p1 = pmid + dds1*Point(toward);
	Point p2 = pmid + dds2*Point(toward);

	midlength = (dds1-dds2)/2;

	/* XXX */
	/* take care! we modify something here! */
	if (midlength>getsegmentMidlenlb && midlength <= ip.size()){
		Segment seg(p1, p2);
		seg.id = segid;
		seg.width = midwidth*2;
		segment[segid] = seg;
	}
}

void CrossCounter::fishEat(UMAP<LL, Segment> &seg){
	cout<<"CrossCounter::fishEat seg.size() = "<<seg.size()<<endl;
	for (auto xt = gridsegxid.begin();
			xt != gridsegxid.end(); xt++){
		for (auto it = xt->second.begin();
				it!=xt->second.end(); it++){
			if (seg.find(*it) == seg.end()) continue;
			Segment &si = seg[*it];
			if (!si.valid) continue;

			double leni = dis(si.p1, si.p2);
			for (auto jt = xt->second.begin();
					jt!=xt->second.end(); jt++){
				if (*it == *jt) continue;
				if (seg.find(*jt) == seg.end()) continue;
				Segment &sj = seg[*jt];
				if (!sj.valid) continue;

				double lenj = dis(sj.p1, sj.p2);
				if (leni < lenj * 2) continue;
				Point midj = (sj.p1 + sj.p2)/2;

				double h1 = disls(sj.p1, si.p1, si.p2);
				double h2 = disls(sj.p2, si.p1, si.p2);
				double hmid = disls(midj, si.p1, si.p2);
				if ((h1 < si.width+10 && h2 < si.width+10)){
					sj.valid = false;
				}
			}
		}
	}
	cleanunvalid();
	cout<<"  CrossCounter::fishEat finish, seg.size() = "
		<<seg.size()<<TimeStamp::gettime()<<endl;
}

void CrossCounter::cleanunvalid(){

	USET<LL> unvalidseg;
	for (auto it = segment.begin();
			it != segment.end(); it++){
		/* XXX: we delete road width large than 60 */
		/*
		if (it->second.width > 300/narrow){
			it->second.valid = false;
		}
		*/

		if (!it->second.valid){
			unvalidseg.insert(it->first);
		}
	}
	for (auto it = unvalidseg.begin();
			it != unvalidseg.end(); it++){
		segment.erase(*it);
	}
	for (auto it = xvisit.begin();
			it != xvisit.end(); it++){
		if (unvalidseg.find(it->second) != unvalidseg.end()){
			IPoint ip = getIPoint(it->first);
			LL xid = getID(ip.x/10, ip.y/10);
			gridsegxid[xid].erase(it->second);
		}
	}

	for (auto it = gridsegxid.begin();
			it != gridsegxid.end(); it++){
		USET<LL> invalid;
		for (auto jt = it->second.begin();
				jt != it->second.end(); jt++){
			if (segment.find(*jt) == segment.end()){
				invalid.insert(*jt);
			}
		}
		for (auto jt = invalid.begin();
				jt != invalid.end(); jt++){
			it->second.erase(*jt);
		}
	}

	vector<LL> emptygridxid;
	for (auto it = gridsegxid.begin();
			it != gridsegxid.end(); it++){
		if (it->second.size()==0){
			emptygridxid.push_back(it->first);
		}
	}
	for (int i = 0; i<emptygridxid.size(); i++){
		gridsegxid.erase(emptygridxid[i]);
	}
}

void CrossCounter::merge(CrossCounter &c){
	int tmpow = overlapWidth;
	for (auto it = c.segment.begin();
			it != c.segment.end(); it++){
		Segment sg = it->second;
		overlapWidth = sg.width/2;
		double ds = dis(sg.p1, sg.p2);
		TaxiPoint fakept;
		fakept.pos = sg.p1*narrow;
		fakept.v = 1;
		fakept.toward = atan2(sg.p2.y - sg.p1.y, sg.p2.x - sg.p1.x)*180/M_PI;
		overlap(fakept, ds*narrow, NULL);
	}
	overlapWidth = tmpow;
}


void CrossCounter::outputsegment(bool append, string filename){
	if (filename.length() == 0){
		filename = "ccsegment";
	}
	const string ccsegment = "data/road/"+filename+".dat";
	std::ios_base::openmode oppmod = fstream::out;
	if (append){
		oppmod |= fstream::app;
	}
	ofstream fout(ccsegment.c_str(), oppmod);
	int validseg = 0;
	for (auto it = segment.begin();
			it != segment.end(); it++) if (it->second.valid){
		validseg++;
		Segment &sg = it->second;
		Point p1 = sg.p1, p2 = sg.p2;
		fout<<p1.x<<'\t'<<p1.y<<'\t'<<p2.x<<'\t'<<p2.y<<'\t'<<(int)sg.width<<' '<<sg.id<<endl;
	}
	cout<<"CrossCounter::outputsegment valid segment = "<<validseg<<endl;
	cout<<"\tCrossCounter::outputsegment finish,"<<TimeStamp::gettime();
}
void CrossCounter::outputTestPoint(){
	const string cctp = "data/road/cctestpoint.dat";
	ofstream fout(cctp.c_str());
	for (auto it = ccounter.begin();
			it != ccounter.end(); it++){
		IPoint ip = getIPoint(it->first);
		if (regionLimit && (
				ip.x < regionLimit1.x || ip.y<regionLimit1.y
				|| ip.x > regionLimit2.x || ip.y > regionLimit2.y)){
			continue;
		}
		Point xp = Point(ip) + Point(getCross(ip).avertow);
		fout<<ip.x<<' '<<ip.y<<' '<<xp.x<<' '<<xp.y<<endl;
	}
	fout.close();
	cout<<"CrossCounter::outputTestPoint finish"<<TimeStamp::gettime();
}


int CrossCounter::towardSeg(Segment s){
	double dx = s.p2.x-s.p1.x;
	double dy = s.p2.y-s.p1.y;
	if (fabs(dx) > fabs(dy)){
		if (dx > 0) return 0;
		else return 2;
	}else{
		if (dy > 0) return 3;
		else return 1;
	}
}

LL CrossCounter::getSegmentIDForw(Point pt, double toward){
	Point tw = Point(toward)*2;
	LL sid = -1;
	int i;
	for (i = 0; i<10; i++){
		sid = getGridxNearestSegmentID(pt);
		if (sid != -1) break;
		pt = pt + tw;
	cout<<i<<endl;
	}
	return sid;
}

LL CrossCounter::getNearestSegmentID(Point pt, int fourtoward){
	bool find = false;
	LL fid = -1;
	double mindis = 1e9;
	for (auto it = segment.begin();
			it != segment.end(); it++){
		Segment &s = it->second;
		double d = disls(pt, s.p1, s.p2);
		if (fourtoward == -1 || (fourtoward&1) == (towardSeg(s)&1)){
			if (!find || mindis > d){
				mindis = d;
				find = true;
				fid = it->first;
			}
		}
	}
	return fid;
}
LL CrossCounter::getSegmentIDUntil(Point pt, int range){
	double mind = 999999999;
	LL fd = -1;
	USET<LL> vs;
	for (int dx = -range; dx <=range; dx++){
		for (int dy = -range; dy<=range; dy++){
			LL pointID = getID((int)pt.x/10 + dx, (int)pt.y/10 + dy);
			if (gridsegxid.find(pointID) == gridsegxid.end()){
				continue;
			}
			USET<LL> &segx = gridsegxid[pointID];
			vs.insert(segx.begin(), segx.end());
		}
	}
	for (auto it = vs.begin();
			it != vs.end(); it++){
		double dd = disls(pt, segment[*it].p1, segment[*it].p2);
		if (dd < mind){
			mind = dd;
			fd = *it;
		}
	}
	if (fd == -1){
		if (range == 0) range = 1;
		return getSegmentIDUntil(pt, range*2);
	}
	return fd;
}

LL CrossCounter::getSegmentID(Point pt){
	return getSegmentID(pt, 0);
}

LL CrossCounter::getSegmentID(Point pt, int range){
	bool find = false;
	LL fid = -1;
	double mindis = 1e9;

	for (int dx = -range; dx <=range; dx++){
		for (int dy = -range; dy<=range; dy++){
			LL pointID = getID((int)pt.x/10 + dx, (int)pt.y/10 + dy);
			if (gridsegxid.find(pointID) == gridsegxid.end()){
				continue;
			}
			USET<LL> &segx = gridsegxid[pointID];
			for (auto it = segx.begin();
					it != segx.end(); it++){
				Segment &s = segment[*it];
				double d = disls(pt, s.p1, s.p2);
				if (d <= s.width){
					if (!find || mindis > d){
						mindis = d;
						find = true;
						fid = *it;
					}
				}
			}
		}
	}
	return fid;
}

LL CrossCounter::getGridxNearestSegmentID(Point pt){
	bool find = false;
	LL fid = -1;
	double mindis = 1e9;

	LL pointID = getID((int)pt.x/10, (int)pt.y/10);
	if (gridsegxid.find(pointID) == gridsegxid.end()){
		return -1;
	}
	USET<LL> &segx = gridsegxid[pointID];
	for (auto it = segx.begin();
			it != segx.end(); it++){
		Segment &s = segment[*it];
		double d = disls(pt, s.p1, s.p2);
		if (!find || mindis > d){
			mindis = d;
			find = true;
			fid = *it;
		}
	}
	cout<<fid<<endl;
	return fid;
}
USET<LL> CrossCounter::getSegmentIDVect(Point pt, int range){
	USET<LL> res;

	for (int dx = -range; dx <=range; dx++){
		for (int dy = -range; dy<=range; dy++){
			LL pointID = getID((int)pt.x/10 + dx, (int)pt.y/10 + dy);
			if (gridsegxid.find(pointID) == gridsegxid.end()){
				continue;
			}
			USET<LL> &segx = gridsegxid[pointID];
			for (auto it = segx.begin();
					it != segx.end(); it++){
				assert(segment.find(*it) != segment.end());
				Segment &s = segment[*it];
				double d = disls(pt, s.p1, s.p2);
				if (d <= s.width){
					res.insert(*it);
				}
			}
		}
	}
	return res;
}
USET<LL> CrossCounter::getSegmentIDVectAll(Point pt, int exwidth){
	USET<LL> res;
	for (auto it = segment.begin();
			it != segment.end(); it++){
		Segment &s = it->second;
		double d = disls(pt, s.p1, s.p2);
		if (d <= s.width + exwidth){
		//if (d <= 1 + exwidth){
			res.insert(it->first);
		}
	}
	return res;
}


void CrossCounter::init(){

}
void CrossCounter::save(){
	string ccfile = "data/road/"+name+".dat";
	ofstream fout(ccfile.c_str(), ios::out|ios::binary);
	fout.write((char*)&gotaxiNightlimit, sizeof(gotaxiNightlimit));
	fout.write((char*)&avertowRange, sizeof(avertowRange));
	fout.write((char*)&avertowEnable, sizeof(avertowEnable));
	fout.write((char*)&bfspointRange, sizeof(bfspointRange));
	fout.write((char*)&bfspointCounterlb, sizeof(bfspointCounterlb));
	fout.write((char*)&getsegmentMidlenlb, sizeof(getsegmentMidlenlb));

	/* save ccounter */
	LL pointnum = ccounter.size();
	fout.write((char*)&pointnum, sizeof(pointnum));
	for (auto it = ccounter.begin();
			it != ccounter.end(); it++){
		fout.write((char*)&*it, sizeof(*it));
	}
	/* save segment */
	LL segnum = segment.size();
	fout.write((char*)&segnum, sizeof(segnum));
	for (auto it = segment.begin();
			it != segment.end(); it++){
		fout.write((char*)&*it, sizeof(*it));
	}
	/* save xvisit */
	/*
	LL visize = xvisit.size();
	fout.write((char*)&visize, sizeof(visize));
	for (auto it = xvisit.begin();
			it != xvisit.end(); it++){
		fout.write((char*)&*it, sizeof(*it));
	}
	*/
	/* save gridsegxid */
	LL gridsize= gridsegxid.size();
	fout.write((char*)&gridsize, sizeof(gridsize));
	for (auto it = gridsegxid.begin();
			it != gridsegxid.end(); it++){
		LL usize = it->second.size();
		fout.write((char*)&(it->first), sizeof(it->first));
		fout.write((char*)&usize, sizeof(usize));
		for (auto jt = it->second.begin();
				jt != it->second.end(); jt++){
			fout.write((char*)&*jt, sizeof(*jt));
		}
	}

	fout.close();
	cout<<"CrossCounter::save ccounter   "<<pointnum<<endl;
	cout<<"CrossCounter::save segment    "<<segnum<<endl;
	//cout<<"CrossCounter::save xvisit     "<<visize<<endl;
	cout<<"CrossCounter::save gridsegxid "<<gridsize<<endl;
	cout<<"\t\tCrossCounter::save finish,"<<TimeStamp::gettime();
}
void CrossCounter::load(){
	load(false);
}
void CrossCounter::load(bool brief){
	string ccfile = "data/road/"+name+".dat";
	cout<<"CrossCounter::load "<<ccfile<<TimeStamp::gettime();
	ifstream fin(ccfile.c_str(), ios::in|ios::binary);

	fin.read((char*)&gotaxiNightlimit, sizeof(gotaxiNightlimit));
	fin.read((char*)&avertowRange, sizeof(avertowRange));
	fin.read((char*)&avertowEnable, sizeof(avertowEnable));
	fin.read((char*)&bfspointRange, sizeof(bfspointRange));
	fin.read((char*)&bfspointCounterlb, sizeof(bfspointCounterlb));
	fin.read((char*)&getsegmentMidlenlb, sizeof(getsegmentMidlenlb));

	LL pointnum;
	fin.read((char*)&pointnum, sizeof(pointnum));
	ccounter.clear();
	vector<pair<LL, Cross> > tmppr;
	if (!brief){
		tmppr.resize(pointnum);
		fin.read((char*)&tmppr[0], pointnum * sizeof(tmppr[0]));
		ccounter.insert(tmppr.begin(), tmppr.end());
	}else{
		fin.seekg(pointnum *sizeof(pair<LL, Cross>), ios::cur);
		pointnum = 0;
	}

	segment.clear();
	LL segnum;
	fin.read((char*)&segnum, sizeof(segnum));
	vector<pair<LL, Segment> > tmpps;
	tmpps.resize(segnum);

	fin.read((char*)&tmpps[0], segnum * sizeof(tmpps[0]));
	segment.insert(tmpps.begin(), tmpps.end());

	xvisit.clear();
	/*
	LL visize;
	fin.read((char*)&visize, sizeof(visize));
	pair<LL, LL> pv;
	for (LL i = 0; i<visize; i++){
		fin.read((char*)&pv, sizeof(pv));
		xvisit.insert(pv);
	}
	fin.seekg(sizeof(pv)*visize, ios::cur);
	*/

	gridsegxid.clear();
	LL gridsize;
	fin.read((char*)&gridsize, sizeof(gridsize));
	if (!brief){
		for (LL i = 0; i<gridsize; i++){
			LL itf;
			fin.read((char*)&itf, sizeof(itf));
			LL usize;
			fin.read((char*)&usize, sizeof(usize));
			if (usize > 0){
				vector<LL> its;
				its.resize(usize);
				fin.read((char*)&its[0], sizeof(its[0])*usize);
				gridsegxid[itf].insert(its.begin(), its.end());
			}
		}
	}else{
		gridsize = 0;
	}

	fin.close();
	cout<<"CrossCounter::load ccounter   "<<pointnum<<endl;
	cout<<"CrossCounter::load segment    "<<segnum<<endl;
	//cout<<"CrossCounter::load xvisit     "<<visize<<endl;
	cout<<"CrossCounter::load gridsegxid "<<gridsize<<endl;
	cout<<"\t\tCrossCounter::load finish,"<<TimeStamp::gettime();
}

void CrossCounter::savecustom(){
	string ccfile = "data/road/"+name+"_custom.dat";
	ofstream fout(ccfile.c_str(), ios::out|ios::binary);
	fout.write((char*)&gotaxiNightlimit, sizeof(gotaxiNightlimit));
	fout.write((char*)&avertowRange, sizeof(avertowRange));
	fout.write((char*)&avertowEnable, sizeof(avertowEnable));
	fout.write((char*)&bfspointRange, sizeof(bfspointRange));
	fout.write((char*)&bfspointCounterlb, sizeof(bfspointCounterlb));
	fout.write((char*)&getsegmentMidlenlb, sizeof(getsegmentMidlenlb));

	/* save segment */
	LL segnum = segment.size();
	fout.write((char*)&segnum, sizeof(segnum));
	for (auto it = segment.begin();
			it != segment.end(); it++){
		fout.write((char*)&*it, sizeof(*it));
	}

	fout.close();
	cout<<"\t\tCrossCounter::savecustom finish,"<<TimeStamp::gettime();
}
void CrossCounter::loadcustom(){
	string ccfile = "data/road/"+name+"_custom.dat";
	cout<<"CrossCounter::load "<<ccfile<<TimeStamp::gettime();
	ifstream fin(ccfile.c_str(), ios::in|ios::binary);

	fin.read((char*)&gotaxiNightlimit, sizeof(gotaxiNightlimit));
	fin.read((char*)&avertowRange, sizeof(avertowRange));
	fin.read((char*)&avertowEnable, sizeof(avertowEnable));
	fin.read((char*)&bfspointRange, sizeof(bfspointRange));
	fin.read((char*)&bfspointCounterlb, sizeof(bfspointCounterlb));
	fin.read((char*)&getsegmentMidlenlb, sizeof(getsegmentMidlenlb));

	segment.clear();
	LL segnum;
	fin.read((char*)&segnum, sizeof(segnum));
	vector<pair<LL, Segment> > tmpps;
	tmpps.resize(segnum);

	fin.read((char*)&tmpps[0], segnum * sizeof(tmpps[0]));
	segment.insert(tmpps.begin(), tmpps.end());
	cout<<"\t\tCrossCounter::loadcustom finish,"<<TimeStamp::gettime();
}

#endif
