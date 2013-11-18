#include "road/cross.h"
#include "road/road.h"
#include "prob/prob.h"
RoadMap r;
CrossCounter cc("ccfinal_lispnt_short");
//Prob prob(cc, "probdebug");
//CrossCounter cc("c2");
//Prob prob(cc, "probfinal_c2");
Prob prob(cc, "probfinal");
void dp(Point p1, Point p2){
	cout<<"***************"<<endl;
	LL sid1 = cc.getSegmentIDUntil(p1/cc.narrow, 1);
	LL sid2 = cc.getSegmentIDUntil(p2/cc.narrow, 1);
	vector<pair<LL, bool> > ths = r.findpath(cc, sid1, sid2);
	//vector<pair<LL, bool> > ths = r.neib[sid1][sid2];
foo:
	p1 = p1 / 3;
	p2 = p2 / 3;
	cout<<" ths size"<<ths.size()<<endl;
	for (int i = ths.size()-1; i<ths.size(); i++){
		ofstream fout("data/debug.out");
		fout<<p1.x<<' '<<p1.y<<' '<<p2.x<<' '<<p2.y<<endl;
		for (int j = 0; j<=i; j++){
			LL id = ths[j].first;
			fout<<cc.segment[id].p1.x<<' '<<cc.segment[id].p1.y<<' '
				<<cc.segment[id].p2.x<<' '<<cc.segment[id].p2.y<<endl;
		}
		fout.close();
		system("./script/debug.py");
	}
}


int main(){
	cc.load();
	cc.outputsegment(false, "");
	r.buildConnect(cc, true);
	ifstream fin("./wxl");
	double x1, y1, x2, y2;
	while(fin>>x1>>y1>>x2>>y2){
		/*
		if (fabs(x1-x2) < 400 || fabs(y1 - y2) < 400){
			continue;
		}
		*/
		cout<<x1<<' '<<y1<<' '<<x2<<' '<<y2<<endl;
		Point p1(x1, y1), p2(x2, y2);
		dp(p1, p2);
	}
	return 0;
}


