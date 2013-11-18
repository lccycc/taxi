#include "taxi/taxi.h"
#include "road/cross.h"
#include "road/road.h"
#include "road/brush.h"
#include "prob/prob.h"
Taxi t("taxifull");
Taxi t2("taxifull");
CrossCounter cc("ccfull");
Prob p(cc, "pfull");
RoadMap r;
int main(){
	t2.readorigindata("data/txt/filelist");
	t.load();
	cc.goTaxi(t, NULL);
	cc.save();
	cc.savecustom();
	//cc.load();
	cc.bfs();
	cc.save();
	cc.outputsegment(false, "");
	p.goTaxi(t, r);
	p.buildcustom();
	return 0;
}
