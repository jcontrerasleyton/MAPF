#ifndef TREEWIDTHWRAPPER_H
#define TREEWIDTHWRAPPER_H

#include <string>
#include <vector>
#include <utility>

using std::vector;
using std::pair;

class GraphAlgoWrapper {
 public:

  // returns the tree-width
  int getTW(int numVars, int numCons, vector<vector<int> > hyperedges);

  // returns the minimum-vertex-cover
  int getMVC(int numVars, vector< pair<int, int> > edges);

};

#endif
