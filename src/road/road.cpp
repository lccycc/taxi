#ifndef ROAD_CPP
#define ROAD_CPP

#include "road.h"

const double eps = 1e-6;
int sgn(double a){
	return a>eps?1:a<-eps?-1:0;
}

bool pntonseg(Point p, Segment s){
	if (sgn(disls(p, s.p1, s.p2))==0){
		return true;
	}
	return false;
}

bool intersect(Segment s1, Segment s2, Point &s){
	double width = (s1.width + s2.width);
	Line l1(s1.p1, s1.p2);
	Line l2(s2.p1, s2.p2);
	if (!intersect(l1, l2, s)){
		return false;
	}
	if (disls(s, s1.p1, s1.p2)<= width && disls(s, s2.p1, s2.p2)<= width){
		return true;
	}
	width += 10;
	double t1 = l1.toward();
	double t2 = l2.toward();
	t2 = justtoward(t2, t1);
	if (angledis(t1, t2) < 10 || fabs(angledis(t1, t2) - 90) < 10){
		s = (s1.p1 + s2.p1)/2;
		if (disls(s, s1.p1, s1.p2)<=width && disls(s, s2.p1, s2.p2)<=width){
			return true;
		}
		s = (s1.p2 + s2.p1)/2;
		if (disls(s, s1.p1, s1.p2)<=width && disls(s, s2.p1, s2.p2)<=width){
			return true;
		}
		s = (s1.p1 + s2.p2)/2;
		if (disls(s, s1.p1, s1.p2)<=width && disls(s, s2.p1, s2.p2)<=width){
			return true;
		}
		s = (s1.p2 + s2.p2)/2;
		if (disls(s, s1.p1, s1.p2)<=width && disls(s, s2.p1, s2.p2)<=width){
			return true;
		}
	}
	return false;
}
double operator *(Point a, Point b){
	return a.x*b.x+a.y*b.y;
}

double distoward(Point s, Point p1, Point p2){
	double d = dis(p1, p2);
	return (s-p1)*(p2-p1)/d;
}

void dfs(int from, int u, vector<vector<int> > &edge, vector<int> &road){
	road.push_back(u);
	if (edge[u].size() != 2){
		return ;
	}
	if (edge[u][0] != from){
		dfs(u, edge[u][0], edge, road);
	}else{
		dfs(u, edge[u][1], edge, road);
	}
}

void RoadMap::ifneib(Point &p1, Point &p2, LL sid, CrossCounter &cc, bool ifpvt){
	for (int i = 0; i<400; i++){
		USET<LL> us = cc.getSegmentIDVectAll(p2, 0);
		us.erase(sid);
		if (us.size() == 0){
			double d = dis(p1, p2);
			p2 = (p2-p1)*(d+1)/d + p1;
			continue;
		}
		double mind = 99999;
		LL id = -1;
		for (auto it = us.begin();
				it != us.end(); it++){
			Point midp = (cc.segment[*it].p1 + cc.segment[*it].p2)/2;
			double d = disls(midp, cc.segment[*it].p1, cc.segment[*it].p2);
			/*
			if (id == -1 || d < mind){
				mind = d;
				id = *it;
			}
			*/
			id = *it;
			neib[sid][id].clear();
			neib[sid][id].push_back(make_pair(id, ifpvt));
			neib[id][sid].clear();
			bool xs = true;
			if (dis(p2, cc.segment[id].p1) > dis(p2, cc.segment[id].p2)){
				xs = false;
			}
			neib[id][sid].push_back(make_pair(sid, xs));
		}
		break;
		/*
		neib[sid][id].clear();
		neib[sid][id].push_back(make_pair(id, ifpvt));
		neib[id][sid].clear();
		bool xs = true;
		if (dis(p2, cc.segment[id].p1) > dis(p2, cc.segment[id].p2)){
			xs = false;
		}
		neib[id][sid].push_back(make_pair(sid, xs));
		break;
		*/
	}
}

vector<pair<LL, bool> > RoadMap::findpath(CrossCounter &cc, LL sid1, LL sid2){
	vector<pair<LL, bool> > res;
	USET<LL> xset;
	if (sid1 == -1 || sid2 == -1){
		return res;
	}
	vector<pair<LL, bool> > vs = neib[sid1][sid2];
	if (vs.size() == 0){
		return res;
	}
	/* start make fake trace */
	vector<Point> fp;
	for (int i = 0; i < vs.size(); i++){
		LL id = vs[i].first;
		assert(cc.segment.find(id) != cc.segment.end());
		fp.push_back((cc.segment[id].p1+cc.segment[id].p2)/2);
	}
	for (int i = 0; i+1<fp.size(); i++){
		double tw1 = getSegTw(cc.segment[vs[i].first]);
		double tw2 = getSegTw(cc.segment[vs[i+1].first]);
		tw1 = justtoward(tw1, tw2);
		if (angledis(tw1, tw2) > 35){
			double tw = getPntPairTw(fp[i], fp[i+1]);
			bool same = Prob::ifpvt_must(cc.segment[vs[i].first], tw);
			if (xset.find(vs[i].first) != xset.end()) continue;
			xset.insert(vs[i].first);
			res.push_back(make_pair(vs[i].first, same));
		}else{
			int mxt = 2;
			for (int j = 0; j<mxt; j++){
				Point p1 = ((mxt-j)*fp[i] + j*fp[i+1])/mxt;
				Point p2 = ((mxt-j-1)*fp[i] + (j+1)*fp[i+1])/mxt;
				double tw = getPntPairTw(fp[i], fp[i+1]);
				USET<LL> qset = cc.getSegmentIDVect(p1, 0);
				for (auto ut = qset.begin();
						ut != qset.end(); ut++){
					LL u = *ut;
					if (xset.find(u) != xset.end()) continue;
					double stw = getSegTw(cc.segment[u]);
					double ntw = justtoward(tw, stw);
					if (angledis(stw, ntw)>40){
						continue;
					}
					bool same = Prob::ifpvt_must(cc.segment[u], tw);
					xset.insert(u);
					res.push_back(make_pair(u, same));
				}
			}
		}
	}
	return res;
}

void RoadMap::findpathAll(CrossCounter &cc){
	int cnt = 0;
	int sz = cc.segment.size();
	for (auto it = cc.segment.begin();
			it != cc.segment.end(); it++){
		for (auto jt = cc.segment.begin();
				jt != cc.segment.end(); jt++){
			LL i = it->first, j = jt->first;
			vector<pair<LL, bool> > res = findpath(cc, i, j);
			fpa[i][j] = res;
			cnt++;
		}
	}
}

void RoadMap::buildConnect(CrossCounter &cc){
	return buildConnect(cc, false);
}
void RoadMap::buildConnect(CrossCounter &cc, bool simple){
	neib.clear();
/*
for (auto it = cc.segment.begin();
		it != cc.segment.end(); it++){
	for (auto jt = cc.segment.begin();
			jt != cc.segment.end(); jt++){
		Point s;
			if (intersect(it->second, jt->second, s)){
				neib[it->first][jt->first].push_back(jt->first);
			}
		}
	}
	*/
	for (auto it = cc.segment.begin();
			it != cc.segment.end(); it++){
		ifneib(it->second.p1, it->second.p2, it->first, cc, true);
		ifneib(it->second.p2, it->second.p1, it->first, cc, false);
	}
	ofstream fout("data/road/roads.dat");
	for (auto it = cc.segment.begin();
			it != cc.segment.end(); it++){
		for (auto jt = cc.segment.begin();
				jt != cc.segment.end(); jt++){
			LL i = it->first, j = jt->first;
			if (neib[i][j].size()){
				Point pf((it->second.p1 + it->second.p2)/2);
				Point pt((jt->second.p1 + jt->second.p2)/2);
				fout<<pf.x<<' '<<pt.x<<endl;
				fout<<pf.y<<' '<<pt.y<<endl;
			}
		}
	}
	fout.close();


	for (auto kt = cc.segment.begin();
			kt != cc.segment.end(); kt++){
		LL k = kt->first;
		for (auto it = cc.segment.begin();
				it != cc.segment.end(); it++){
			LL i = it->first;
			for (auto jt = cc.segment.begin();
					jt != cc.segment.end(); jt++){
				LL j = jt->first;
				if (neib[i][k].size() &&
						neib[k][j].size() &&
						(neib[i][j].size() == 0 ||
						 neib[i][j].size() >
							 neib[i][k].size() + neib[k][j].size())){
					neib[i][j] = neib[i][k];
					neib[i][j].insert(neib[i][j].end(), neib[k][j].begin(), neib[k][j].end());
				}
			}
		}
	}
	for (auto it = cc.segment.begin();
			it != cc.segment.end(); it++){
		LL i = it->first;
		for (auto jt = cc.segment.begin();
				jt != cc.segment.end(); jt++){
			LL j = jt->first;
			if (neib[i][j].size()){
				LL id = neib[i][j][0].first;
				bool ifp = !neib[j][i].back().second;
				neib[i][j].insert(neib[i][j].begin(), make_pair(i, ifp));
			}
		}
	}

	if (!simple){
		findpathAll(cc);
	}
}


void RoadMap::build(UMAP<LL, Segment> &segmap){
	cout<<"RoadMap::build begin"<<TimeStamp::gettime();
	vector<Segment> seg;
	vector<Point> cp;
	vector<vector<int> > crs;
	for (auto it = segmap.begin();
			it != segmap.end(); it++){
		seg.push_back(it->second);
		crs.push_back(vector<int>());
	}
	for (int i = 0; i<seg.size(); i++){
		for (int j = i+1; j<seg.size(); j++){
			Point s;
			if (!intersect(seg[i], seg[j], s)){
				continue;
			}
			int id = cp.size();
			cp.push_back(s);
			crs[i].push_back(id);
			crs[j].push_back(id);
		}
	}
	for (int i = 0; i<seg.size(); i++){
		if (crs[i].size() >= 2) continue;
		if (crs[i].size() == 0){
			crs[i].push_back(cp.size());
			cp.push_back(seg[i].p1);
			crs[i].push_back(cp.size());
			cp.push_back(seg[i].p2);
		}
		if (crs[i].size() == 1){
			Point s = cp[crs[i][0]];
			Point sp;
			if (dis(s, seg[i].p1) > dis(s, seg[i].p2)){
				sp = seg[i].p1;
			}else{
				sp = seg[i].p2;
			}
			crs[i].push_back(cp.size());
			cp.push_back(sp);
		}
	}

	for (int i = 0; i<seg.size(); i++){
		for (int j = 0; j<crs[i].size(); j++){
			for (int k = j+1; k<crs[i].size(); k++){
				double dsj = distoward(cp[crs[i][j]], seg[i].p1, seg[i].p2);
				double dsk = distoward(cp[crs[i][k]], seg[i].p1, seg[i].p2);
				if (dsk < dsj){
					swap(crs[i][j], crs[i][k]);
				}
			}
		}
	}

	vector< vector<int> > edge;
	for (int i = 0; i<cp.size(); i++){
		edge.push_back(vector<int>());
	}
	for (int i = 0; i<seg.size(); i++){
		for (int j = 0; j<crs[i].size(); j++){
			int id = crs[i][j];
			if (j > 0){
				edge[id].push_back(crs[i][j-1]);
			}
			if (j + 1 < crs[i].size()){
				edge[id].push_back(crs[i][j+1]);
			}
		}
	}
	vector<USET<int> > visit;
	for (int i = 0; i<cp.size(); i++){
		visit.push_back(USET<int>());
	}

	roads.clear();
	for (int i = 0; i< cp.size(); i++){
		if (edge[i].size() != 2){
			for (int j = 0; j<edge[i].size(); j++){
				if (visit[i].find(edge[i][j]) != visit[i].end()){
					continue;
				}
				vector<int> road;
				road.push_back(i);
				dfs(i, edge[i][j], edge, road);
				visit[road[road.size()-1]].insert(road[road.size()-2]);
				Road r;
				for (int k = 0; k<road.size(); k++){
					r.pts.push_back(cp[road[k]]);
				}
				roads.push_back(r);
			}
		}
	}
	cout<<"\t\tRoadMap::build finish"<<TimeStamp::gettime();
}
void RoadMap::outputroad(){
	cout<<"RoadMap::outputroad begin, roads.size() =  "<<roads.size()<<endl;;
	ofstream fout("data/road/roads.dat");
	for (int i = 0; i < roads.size(); i++){
		vector<Point> &pts = roads[i].pts;
		for (int j = 0; j<pts.size(); j++){
			fout<<pts[j].x<<'\t';
		}
		fout<<endl;
		for (int j = 0; j<pts.size(); j++){
			fout<<pts[j].y<<'\t';
		}
		fout<<endl;
	}
	fout.close();
	cout<<"\t\tRoadMap::outputroad finish"<<TimeStamp::gettime();
}

#endif
