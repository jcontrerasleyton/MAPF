#ifndef SCOMP_H
#define SCOMP_H
#include <iostream>
#include <vector>
using namespace std;

struct SComp{
	vector<int> cut;
	int firstElem;
	int sum; // used only in cache identification
	void Print(){
		cerr<<"SComp: has "<<cut.size()<<" cut-vars,"<<"  firstElem:"<<firstElem<<endl;
		cerr<<"\t"<<"cut-vars:";
		for(vector<int>::iterator itr=cut.begin();itr!=cut.end();itr++)
			cerr<<" "<<*itr;
		cerr<<endl;		
	}

};

struct SCompVarsPair{
	SComp sComp;
	vector<int> vars;

};


#endif

