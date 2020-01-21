#ifndef CMINDEGTREEDECOMPOSER_H
#define CMINDEGTREEDECOMPOSER_H

#include "CConstraintHypergraph.h"
#include "CTreeDecomposition.h"
#include <vector>
using namespace std;

class CMinDegTreeDecomposer{
	private:
		CConstraintHypergraph* cchg;
		CTreeDecomposition* ctd;
		int MinDegWidth; // this is also a treeWidth, unlike MCS width
		vector<int> MinDegOrder;
	public:
		CMinDegTreeDecomposer(CConstraintHypergraph* chg, CTreeDecomposition* td){ 
			cchg=chg; ctd=td;
		}
		int GetMinDegWidth(){return MinDegWidth;} 
		void ObtainMinDegOrder(); //obtains the MinDeg order
		vector<int> GetMinDegOrder(){return MinDegOrder;}
		void Decompose();   //obtains a MinDeg width, then obtains tree decomposition in "ctd"
		void PrintMinDegOrder();

};
#endif
