#ifndef CHYPERTREENODE_H
#define CHYPERTREENODE_H

#include <vector>
#include <map>
using namespace std;
#include "SComp.h"
#include "CDatabase.h"
#include "CConstraintHypergraph.h"
#include "cputime.h"
#include "Sort.h"

//TODO: Dump the constraint graph of connCons and check the width of its tree decomposition
	// You can think of noGoods internal to each CHTNode, attached to specific decision levels and 
			//consisting of markedRCVars [+-] markedOtherVars
	// Is watching necessary or is it just an overhead. Check the typical size of the "cons" lists and think!

class CHypertreeNode{
	private:
		SCompVarsPair sCompVarsPair;
		vector<int> currKString;
		vector<int> connCons;// the vector of constraint that cover the "cv" variables
		map<int,vector<int> > var2cons; // contains locs in connCons vector
		vector<int> markVars;//do some kind of encoding to check "rc" covering
		int markedRCVars;
		int markedOtherVars;
		int totalImpliedCons;
		bool rootHack;
		
		vector<int> alpha;

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
		
		static vector<vector<int> > size2Cons;
		static int* tempInt; 
		
		static bool connectedHTD; // prefer connectedHTDecomposer or normal HTDecomposer, default true
		static bool isomorphicCompDetect; // prefer isomorphicCompDetection, default true

		static bool timeLimitedStop; // when the root CHTNode call exists, this value is true iff there was a time out
		static double startTime;
		static double timeLimit; // 260000 seconds by default

		CHypertreeNode(SCompVarsPair sCVP);
		~CHypertreeNode();
		bool Decompose();
		void SetAlpha(vector<int>& alphaIn) {alpha=alphaIn;}
};
#endif
