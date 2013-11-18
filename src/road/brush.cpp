#ifndef BRUSH_CPP
#define BRUSH_CPP

#include "brush.h"

Brush::Brush(string _name){
	name = _name;
}

void Brush::getrest(Taxi &t, CrossCounter &c){
	for (auto it = t.offset.begin();
			it != t.offset.end(); it++){
		TaxiTrace tr;
		t.getTaxiTrace(it->first, tr);
		getrest(tr, c);
	}
	cout<<"Brush::getrest rest.size = "<<rest.size()<<endl;
}
void Brush::getrest(TaxiTrace &tr, CrossCounter &c){
	for (auto jt = tr.begin();
			jt != tr.end(); jt++){
		TaxiPoint &taxipt = *jt;

		if (taxipt.v >= 2){
			if (c.getSegmentID(taxipt.pos/c.narrow) == -1){
				rest.push_back(taxipt);
			}
		}
	}
}
void Brush::outputrestpnt(){
	string restpngdat = "data/road/restpng.dat";
	ofstream fout(restpngdat.c_str());
	for (int i = 0; i<rest.size(); i++){
		double dx = acos(rest[i].toward*M_PI/180);
		double dy = asin(rest[i].toward*M_PI/180);
		fout<<rest[i].pos.x/3<<' '<<rest[i].pos.y/3<<' '
			<<(rest[i].pos.x+rest[i].v*2*dx)/3<<' '
			<<(rest[i].pos.y+rest[i].v*2*dy)/3<<endl;
	}
	fout.close();
}
void Brush::save(){
	string bfile = "data/road/" + name + ".dat";
	ofstream fout(bfile.c_str(), ios::out|ios::binary);
	int restsize = rest.size();
	fout.write((char*)&restsize, sizeof(int));
	if (restsize){
		fout.write((char*)&rest[0], sizeof(rest[0])*restsize);
	}
	fout.close();
	cout<<"Brush::save rest.size = "<<rest.size()<<endl;
}
void Brush::load(){
	string bfile = "data/road/"+name+".dat";
	ifstream fin(bfile.c_str(), ios::in|ios::binary);
	int restsize;
	fin.read((char*)&restsize, sizeof(int));
	rest.resize(restsize);
	if (restsize){
		fin.read((char*)&rest[0], sizeof(rest[0])*restsize);
	}
	fin.close();
	cout<<"Brush::load rest.size = "<<rest.size()<<endl;
}
#endif
