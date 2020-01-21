#ifndef COPTTREENODE_H
#define COPTTREENODE_H

#include <vector>
#include <map>
using namespace std;
#include "SComp.h"
#include "CDatabase.h"
#include "CConstraintHypergraph.h"
#include "cputime.h"
#include "Sort.h"

class COptTreeNode{
	private:
		SCompVarsPair sCompVarsPair; // the input connected component
		vector<int> newCut; 
		vector<int> markVars; // -v if the var is in the comp or the cut (cv) , else -2 
        vector<int> candVars; // the candidate variables, each one's  elimination needs to be considered
	public:
		static long creationCount;
		static CConstraintHypergraph* cchg;
		static int numVars;
		static int numCons;
		static int k;
		static vector<vector<int> > ches;
		static vector<vector<int> > var2ches;
		static CDatabase cngd;
		static CDatabase cgd;
		static long callCount;
		static bool recordGoods;
		static bool verbose;
		static bool debug;
		static vector<int> isTouchedVar; // default is 0
		static vector<int> touchedVars;
		static vector<int> isTouchedCon; // default is 0
		static vector<int> touchedCons;

		static int* tempInt;
		static bool connectedOTD; // prefer connected branching
		static bool timeLimitedStop; // when the root COTNode call exists, this value is true iff there was a time out
		static double startTime;
		static double timeLimit; // 260000 seconds by default

		COptTreeNode(SCompVarsPair sCVP);
		~COptTreeNode();
		bool Decompose();
};
#endif
