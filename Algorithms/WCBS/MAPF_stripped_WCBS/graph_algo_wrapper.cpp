#include "graph_algo_wrapper.h"
#include "TreeD/treed.h"
#include "numvc.h"


int GraphAlgoWrapper::getTW(int numVars, int numCons, vector<vector<int> > hyperedges) {
  CConstraintHypergraph cchg;
  cchg.SetHypergraph(numVars, numCons, hyperedges);
  CTreeDecomposition ctd(&cchg);
  //  cout<<"NumVar: "<<cchg.GetNumVars()<<"  NumCons: "<<cchg.GetNumCons()<<endl;
  CMinDegTreeDecomposer mindegtd(&cchg,&ctd);
  mindegtd.Decompose();  // breaks here for RRR_IT=2
  //  ctd.Print();
  return ctd.GetTreeWidth();
}


int GraphAlgoWrapper::getMVC(int numVars, vector< pair<int, int> > edges) {
  build_instance(numVars, edges);  // numvc
  return getVCSize(1, 0, 1);  // numvc
}
