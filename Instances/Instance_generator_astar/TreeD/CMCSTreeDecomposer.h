#ifndef CMCSTREEDECOMPOSER_H
#define CMCSTREEDECOMPOSER_H

#include "CConstraintHypergraph.h"
#include "CTreeDecomposition.h"
#include <vector>
using namespace std;
struct countPair{
	int count;
	int loc;
};


class CMCSTreeDecomposer{
	private:
		CConstraintHypergraph* cchg;
		CTreeDecomposition* ctd;
		int MCSWidth;
		vector<int> MCSOrder;
	public:
		CMCSTreeDecomposer(CConstraintHypergraph* chg, CTreeDecomposition* td){ 
			cchg=chg; ctd=td;
		}
		int GetMCSWidth(){return MCSWidth;} 
		void ObtainMCSOrder(); //obtains the MCS order
		vector<int> GetMCSOrder(){return MCSOrder;}
		void Decompose();   //given an MCS width, obtains tree decomposition in "ctd"
		void PrintMCSOrder();

};
#endif
