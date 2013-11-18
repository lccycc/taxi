#include "taxi/taxi.h"
#include "road/cross.h"
#include "road/road.h"
#include "road/brush.h"
#include "prob/prob.h"
Taxi t("taxifinal");
//CrossCounter cc("ccfinal_lispnt_short_moreroad");
CrossCounter cc("ccfinal_lispnt_short");
Brush brush1("brush1");
CrossCounter c2("c2");
CrossCounter cmerge("cmerge");
/*
*/
Prob p(cc, "probfinal_road");
Prob p2(c2, "probfinal_c2");
//Prob p2(cmerge, "probfinal_roadbreakhalf");
RoadMap r;

int main(){
	//t.readorigindata("data/txt/filelist");
	//t.load();
	/*
	cc.regionLimit = true;
	cc.goTaxi(t, NULL);
	cc.save();
	//cc.load();
	cc.bfs();
	cc.save();
	cc.outputsegment(false, "");
	*/
	/*
	cc.load();
	cmerge.bfspointCounterlb = 1;
	//cmerge.bfspointHeartdis/=2;
	cmerge.merge(cc);
	cmerge.bfs();
	cmerge.save();
	cmerge.outputsegment(false, "");
	*/
	/*
	c2.regionLimit = true;
	c2.bfspointCounterlb = 100;
	c2.bfspointHeartdis*=2;
	c2.goTaxi(t, NULL);
	c2.bfs();
	c2.save();
	//c2.load();
	c2.outputsegment(false, "");
	return 0;
	*/
	/*
	cc.load();
	brush1.getrest(t, cc);
	brush1.save();
	brush1.load();
	//c2.bfspointCounterlb = 10;
	c2.goTaxiTrace(brush1.rest, &cc);
	c2.bfs();
	cc.outputsegment(false, "");
	c2.outputsegment(true, "");
	*/
	/*
	c2.load();
	c2.outputsegment(false, "");
	//r.buildConnect(c2);
	p2.goTaxi(t, r);
	p2.save();
	*/
	/*
	c2.load(true);
	c2.savecustom();
	p2.load();
	p2.buildcustom();
	return 0;
	p2.test();
	*/
	c2.loadcustom();
	p2.loadcustom();
	p2.term();
	return 0;

	/*
	*/
	/*
	cc.load(true);
	cc.outputsegment(false, "");
	*/

	/*
	cc.load();
	p.goTaxi(t);
	p.save();
	*/

	/*
	cc.load();
	cc.outputsegment(false, "");
	r.buildConnect(cc);
	p.goTaxi(t, r);
	p.save();
	*/
	//r.outputroad();
	/*
	cc.load(true);
	cc.outputsegment(false, "");
	p.load();
	p.test();
	*/




	return 0;
}
