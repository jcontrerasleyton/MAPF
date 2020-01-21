#ifndef COPTTREEDECOMPOSER_H
#define COPTTREEDECOMPOSER_H

#include "COptTreeDecomposition.h"
#include "CConstraintHypergraph.h"
#include "COptTreeNode.h"
#include "cputime.h"

class COptTreeDecomposer{
	CConstraintHypergraph *cchg;
	double startTime,usedCPUTime;
	void Init();

	public:
		COptTreeDecomposer(CConstraintHypergraph* chg);
		bool Decompose(int k, double timeLimit=260000);
		void PrintStats();
		bool OptDecompose(double timeLimit=260000,int startWidth=-1);
		double GetUsedCPUTime(){return usedCPUTime;}
};

#endif
