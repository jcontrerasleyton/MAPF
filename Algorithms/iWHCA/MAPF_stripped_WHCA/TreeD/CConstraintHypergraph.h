#ifndef CCONSTRAINTHYPERGRAPH_H
#define CCONSTRAINTHYPERGRAPH_H

#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <utility>
using namespace std;
#include <stdlib.h>
#include "SComp.h"
#include "Sort.h"

class CConstraintHypergraph{
	private:
		int numVars;
		int numCons;
		vector<vector<int> > hyperedges;
		vector<vector<int> > var2Hyperedges;

		vector<int> isTouchedVar; // -2 default
		vector<int> touchedVars;
		vector<int> isTouchedCon; // -2 default
		vector<int> touchedCons;
		queue<int> varQ,conQ;
		vector<int> isPushedVar; // 0 default
		vector<int> pushedVars;

		vector<int> con2Count; // 0 default
		vector<vector<int> > count2Cons;
		vector<int> isTouchedCount; // 0 default
		vector<int> touchedCounts;

		bool debug;
		int* tempCounts;
	public:
		CConstraintHypergraph(){
			numVars=-1; numCons=-1; debug=true;
		}
		void SetHypergraph(int numVarsIn, int numConsIn, 
			vector<vector<int> > hyperedgesIn);

		CConstraintHypergraph(char* filename){ReadFile(filename);}
		void ReadFile(char* filename);
		void Print();
		void PrintMinMaxArity();
		int GetNumVars(){return numVars;}
		int GetNumCons(){return numCons;}
		vector<int>& GetHyperegde(int i){ return hyperedges[i];}
		vector<vector<int> >& GetHyperedges(){return hyperedges;}
		vector<vector<int> >& GetVar2Hyperedges(){return var2Hyperedges;}

		void InitComponentDetection();
		pair<vector<SComp>, vector<vector<int> > >  GetConnectedComponentsPairs(SComp sComp, vector<int> newCut);
		vector<SCompVarsPair>  GetConnectedComponents(SComp sComp, vector<int> newCut);
		vector<int> GetConnectedConstraints(int var); // Get the constraints belonging to the same component as var
		vector<int> GetCoveringConstraints(vector<int>& vars, vector<int>& compVars, bool descendOrder=false); // Get the constraints , each one covering at least one variable in *vars*
		// TODO: --Check whether the ordering of constraints by the above function has any impact...may be a choice for a heuristic
		//		 --Also some constraints in the output might all just cover a single variable, in which case, just one of them is necessary
		//	     --Also check for subsumed clauses
		vector<int> GetConnectedComponentsVars(); // returns a vector of variables, each variable in the vector is the least variable in each
										      //connected component of the hypergraph
		vector<int> GetFullyCoveredConstraints(vector<int>& vars); // returns a vector of cons, each con fuly covered by vars
		vector<int> GetTouchingConstraints(vector<int>& vars, vector<int>& newVars); // returns a vector of cons, each con touched by at least one var in vars,
							// used in normal HypertreeDecomposer (when CHTN::connectedHTD is false)
		vector<SCompVarsPair> GetConnectedComponents(); 

		void ExportDBAI(char* filename);
		void ExportGraph(char* filename);
		void SetDebug(bool flag){debug=flag;}
		vector<int> GetVars(vector<int> lambda);
		vector<int> GetChi(vector<int> lambda, SCompVarsPair sCVP);
};

#endif

