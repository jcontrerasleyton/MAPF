#ifndef CMINFILLDEGTREEDECOMPOSER_H
#define CMINFILLDEGTREEDECOMPOSER_H

#include "CConstraintHypergraph.h"
#include "CTreeDecomposition.h"
#include <vector>
using namespace std;

class CMinFillDegTreeDecomposer{
	private:
		CConstraintHypergraph* cchg;
		CTreeDecomposition* ctd;
		int MinFillDegWidth; // this is also a treeWidth, unlike MCS width
		vector<int> MinFillDegOrder;
	public:
		CMinFillDegTreeDecomposer(CConstraintHypergraph* chg, CTreeDecomposition* td){
			cout<<"WARNING : check the correctness of minfilldeg heuristic "<<endl;
			cout<<"    usage of heaps ?? , "<<endl;
			cchg=chg; ctd=td;
		}
		int GetMinFillDegWidth(){return MinFillDegWidth;}
		void ObtainMinFillDegOrder(); //obtains the MinFillDeg order
		vector<int> GetMinFillDegOrder(){return MinFillDegOrder;}
		void Decompose();   //obtains a MinFillDeg width, then obtains tree decomposition in "ctd"
		void PrintMinFillDegOrder();

};
#endif
