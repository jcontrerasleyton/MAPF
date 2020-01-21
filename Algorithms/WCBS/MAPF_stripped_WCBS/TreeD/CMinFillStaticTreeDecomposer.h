#ifndef CMINFILLSTATICTREEDECOMPOSER_H
#define CMINFILLSTATICTREEDECOMPOSER_H

#include "CConstraintHypergraph.h"
#include "CTreeDecomposition.h"
#include <vector>
using namespace std;

class CMinFillStaticTreeDecomposer{
	private:
		CConstraintHypergraph* cchg;
		CTreeDecomposition* ctd;
		int MinFillStaticWidth; // this is also a treeWidth, unlike MCS width
		vector<int> MinFillStaticOrder;
	public:
		CMinFillStaticTreeDecomposer(CConstraintHypergraph* chg, CTreeDecomposition* td){
			//cout<<"WARNING : check the correctness of minfill heuristic "<<endl;
			cchg=chg; ctd=td;
		}
		int GetMinFillStaticWidth(){return MinFillStaticWidth;}
		void ObtainMinFillStaticOrder(); //obtains the MinFillStatic order
		vector<int> GetMinFillStaticOrder(){return MinFillStaticOrder;}
		void Decompose();   //obtains a MinFillStatic width, then obtains tree decomposition in "ctd"
		void PrintMinFillStaticOrder();

};
#endif
