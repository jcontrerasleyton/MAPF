#ifndef CTABUSEARCH_H
#define CTABUSEARCH_H

#include "CConstraintHypergraph.h"
#include "CTreeDecomposition.h"
#include <vector>
using namespace std;

struct STabuTreeNode{
	int parent; // if root then -1, else index to the parent node
	vector<int> childNodes; // indices to child tree nodes
	int elimVar; // variable whose elimination resulted in this tree node creation
	//vector<int> secondaryElimVars; ?? this may be necessary
	vector<int> otherVars;  //contained other variables
	vector<int> coveredCons; //constraits covered by the treeNode

	// new items, other than those in STreeNode
	long mySubTreeWidth;
	long mySubTreeQuadSum;
	long parentalWidth;
	long parentalQuadSum;
};

struct SWidthSumPair{
	int width;
	long sum;
};

struct STabuPair{
	int var;
	long iteration;
};

class CTabuSearch{
	private:
		CTreeDecomposition* ctd;
		CConstraintHypergraph* cchg;
		vector<STabuTreeNode>  sTabuTreeNodes;
		vector<int> var2Loc; // to store the location of a variable's elimination node
		vector<bool> isPresent;
		vector<bool> isEliminated;
		vector<bool> isCovered;
		
		int numVars; 
		int numCons;

		float tabuListSizeP; // tabuListSize as percentage of the numVars
		int tabuListSize;
		vector<vector<STabuPair> > sTabuPairs;
 		int cpuTimeLimit; 
		int bestWidth;
		long maxIterations;
		double usedCpuTime;
		long diversifyCalls;
		long iteration;

		int root; // to store the location of 'the' root node in sTabuTreeNodes
    public:
		CTabuSearch(CTreeDecomposition* cctd);
		void Print();
		void PrintStats();
		void Dummy();
		void RIPNodeCheck(int node);
		void Solve(float tabuListSizePIn, long maxIterationsIn, int cpuTimeLimitIn);
		int GetBestWidth(){return bestWidth;}
		vector<int> GetElimOrder(); 
	private:
		void GetElimOrder(int loc, vector<int>& currElimOrder);
		void FindSubTreeValues(int i);
		void UpdateParentalValues(int i);
		SWidthSumPair Evaluate(int var); // Evaluate the cost of moving up the var
		void MoveUp(int var); // to move the elimination of 'var' up in the elimination tree
		void RIPCheck(); // Running Intersection Property Checker

		bool IsTabu(int var);
		bool IsSpecialTabu(int var);
		void AddTabu(int var);

		void Diversify(); 
};

#endif
