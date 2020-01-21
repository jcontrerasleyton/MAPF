#ifndef COPTTREEDECOMPOSITION_H
#define COPTTREEDECOMPOSITION_H

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <string>
using namespace std;
#include "CConstraintHypergraph.h"
#include "CDatabase.h"
#include "COptTreeNode.h"

struct SOptTreeNode{
	int parent;
	SComp sComp;
	vector<int> chi; // vars of the node
	vector<int> children;

	void Print(){
		cout<<"Node:"<<endl;
		cout<<"\t"<<"parent:"<<parent<<endl;
		if(children.size()!=0){
			cout<<"\t"<<"children:";
			for(vector<int>::iterator itr=children.begin();itr!=children.end();itr++)
				cout<<" "<<*itr;
			cout<<endl;
		}
		else cout<<"Leaf node"<<endl;
		cout<<"\t"<<"vars:";
		for(vector<int>::iterator itr=chi.begin();itr!=chi.end();itr++)
			cout<<" "<<*itr;
		cout<<endl<<endl;
	}
};

class COptTreeDecomposition{
	private:
		CConstraintHypergraph* cchg;
		CDatabase* cgd;
		vector<SOptTreeNode> nodes;
		int treeWidth;
		vector<int> rootNodes; // vector of root nodes in a OptDecomposition

		vector<bool> isEliminated;
		vector<bool> isCovered;
		vector<int> isTouchedVar; vector<int> touchedVars;
	public:
		COptTreeDecomposition(CConstraintHypergraph* chgI, CDatabase* cgdI);
		void PrintDot(char* filename); // prints a DOT file
		void Print(); //prints in standard out
		void PrintStats(); // just prints the tree- width
		int GetWidth(){ return treeWidth;}
		CConstraintHypergraph* GetConstraintHypergraph(){return cchg;}

		bool RIPCheck(); // checking the running intersection property;
		void RIPCheckNode(int node);
};


#endif
