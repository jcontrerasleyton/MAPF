#ifndef CTREEDECOMPOSER_H
#define CTREEDECOMPOSER_H

#include "CTreeDecomposition.h"

class CTreeDecomposer{
	protected:
		CTreeDecomposition ctd; // a tree decomposition 
	
	public:
		CTreeDecomposer(CTreeDecomposition td){
			ctd=td;
		}
		void PrintTreeDot(char* filename){ ctd.PrintTreeDot(filename);	}
		void PrintTree(){ ctd.PrintTree(); }
		int GetTreeWidth(){	return ctd.GetTreeWidth();	}

		void Decompose(){ //needs to be implemented in each derived decomposer
		}    
};

#endif
