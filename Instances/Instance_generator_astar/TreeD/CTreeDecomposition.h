#ifndef CTREEDECOMPOSITION_H
#define CTREEDECOMPOSITION_H

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
using namespace std;
#include "STreeNode.h"
#include "CConstraintHypergraph.h"

class CTreeDecomposition{
	private:
		CConstraintHypergraph* cchg;
		vector<STreeNode> sTreeNodes;
		int treeWidth;
		int numVars; int numCons;
		vector<int> rootNodes;

		vector<int> elimOrder;  //eliminationOrder

		vector<bool> isPresent;
		vector<bool> isEliminated;
		vector<bool> isCovered;


	public:

		CTreeDecomposition(CConstraintHypergraph* chg);
		void PrintDot(char* filename); // prints a DOT file
		void Print(); //prints in standard out

		int GetNumVars(){return numVars;};
		int GetTreeWidth(){ return treeWidth;}
		vector<int> GetElimOrder(){return elimOrder;}
		STreeNode& GetTreeNode(int i){ return sTreeNodes[i];}
		vector<STreeNode>& GetTreeNodes(){return sTreeNodes;}
 		int AddTreeNode(STreeNode stn); //Add a tree node, as a root
		void ElimOrder2TreeDe(); //Obtain a Tree Decomposition pertaining to the elimOrder
		void SetElimOrder(vector<int>& varOrder);
		void ValidateElimOrder();
		CConstraintHypergraph* GetConstraintHypergraph(){return cchg;}
		void SimpleSifting();

		void RemoveRedundantNodes(); // removes the nodes who are covered by one of their child nodes
		void DeleteDefunctNodes(); // deletes the defunct nodes from sTreeNodes

		void UpdateRootNodes();
		vector<int>& GetRootNodes(){ return rootNodes; }
		void RIPCheck(); // for checking running intersection property
		void RIPNodeCheck(int node);

};


#endif


