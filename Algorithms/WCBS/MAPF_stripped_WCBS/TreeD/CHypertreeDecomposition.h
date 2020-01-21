#ifndef CHYPERTREEDECOMPOSITION_H
#define CHYPERTREEDECOMPOSITION_H

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <string>
using namespace std;
#include "CConstraintHypergraph.h"
#include "CDatabase.h"
#include "CHypertreeNode.h"

struct SHTreeNode{
	int parent; 
	SComp sComp;
	vector<int> lambda;
	vector<int> chi;
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
		cout<<"\t"<<"lambda:";
		for(vector<int>::iterator itr=lambda.begin();itr!=lambda.end();itr++)
			cout<<" "<<*itr;
		cout<<endl;
		cout<<"\t"<<"chi:";
		for(vector<int>::iterator itr=chi.begin();itr!=chi.end();itr++)
			cout<<" "<<*itr;
		cout<<endl<<endl;
	}
};

class CHypertreeDecomposition{
	private:
		CConstraintHypergraph* cchg; 
		CDatabase* cgd;
		vector<SHTreeNode> nodes;
		int width; // hyper- width
		int treeWidth; 
		vector<int> rootNodes; // vector of root nodes in a HT Decomposition
	
		vector<bool> isEliminated;
		vector<bool> isCovered;
		vector<int> isTouchedVar; vector<int> touchedVars;
	public:
		CHypertreeDecomposition(CConstraintHypergraph* chgI, CDatabase* cgdI); 
		void PrintDot(char* filename); // prints a DOT file
		void Print(); //prints in standard out
		void PrintStats(); // just prints the hyper- and tree- width
		void PrintTreeWidth(); // prints the tree- width
		int GetWidth(){ return width;}
		CConstraintHypergraph* GetConstraintHypergraph(){return cchg;}
		
		bool RIPCheck(); // checking the running intersection property;
		void RIPCheckNode(int node);
		bool CheckWidth(); // check whether chi's are covered by lambda's and check the  width variable

};


#endif


