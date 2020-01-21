#ifndef CMINFILLTREEDECOMPOSER_H
#define CMINFILLTREEDECOMPOSER_H

#include "CConstraintHypergraph.h"
#include "CTreeDecomposition.h"
#include "cputime.h"
#include <vector>
using namespace std;

class CMinFillTreeDecomposer{
	private:
		CConstraintHypergraph* cchg;
		CTreeDecomposition* ctd;
		int MinFillWidth; // this is also a treeWidth, unlike MCS width
		vector<int> MinFillOrder;
	public:
		CMinFillTreeDecomposer(CConstraintHypergraph* chg, CTreeDecomposition* td){
			//cout<<"WARNING : check the correctness of minfillstatic heuristic "<<endl;
			cchg=chg; ctd=td;
		}
		int GetMinFillWidth(){return MinFillWidth;}
		void ObtainMinFillOrder(); //obtains the MinFill order
		vector<int> GetMinFillOrder(){return MinFillOrder;}
		void Decompose();   //obtains a MinFill width, then obtains tree decomposition in "ctd"
		void PrintMinFillOrder();

};
#endif
