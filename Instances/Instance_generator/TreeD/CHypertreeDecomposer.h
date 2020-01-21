#ifndef CHYPERTREEDECOMPOSER_H
#define CHYPERTREEDECOMPOSER_H

#include "CHypertreeDecomposition.h"
#include "CConstraintHypergraph.h"
#include "CHypertreeNode.h"
#include "cputime.h"

class CHypertreeDecomposer{
	CConstraintHypergraph *cchg;	
	double startTime,usedCPUTime;
	void Init();

	public:
		CHypertreeDecomposer(CConstraintHypergraph* chg);
		bool Decompose(int k, double timeLimit=260000);
		void PrintStats();
		bool OptDecompose(double timeLimit=260000,int startWidth=-1);
		double GetUsedCPUTime(){return usedCPUTime;}
};

#endif
