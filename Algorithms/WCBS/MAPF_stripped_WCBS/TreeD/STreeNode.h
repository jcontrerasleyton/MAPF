#ifndef STREENODE_H
#define STREENODE_H

#include <vector>
using namespace std;

struct STreeNode{
	int parent; // if root then -1, else index to the parent node
	vector<int> childNodes; // indices to child tree nodes
	int elimVar; // variable whose elimination resulted in this tree node creation
	//vector<int> secondaryElimVars; ?? this may be necessary
	vector<int> otherVars;  //contained other variables
	vector<int> coveredCons; //constraits covered by the treeNode
};

#endif

