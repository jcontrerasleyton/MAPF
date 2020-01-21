#include "CMinFillTreeDecomposer.h"

struct fillPair{
	int fill;	int loc;
};


void CMinFillTreeDecomposer::ObtainMinFillOrder(){
	// read this function carefully before you compile
	int numVars=cchg->GetNumVars();
	vector<vector<int> >& var2Hes=cchg->GetVar2Hyperedges();
	vector<vector<int> >&  hes=cchg->GetHyperedges();
	vector<bool> isPresent;	isPresent.resize(numVars);
	for(int i=0;i<numVars;i++) 	isPresent[i]=false;

	int numSingleOccurrences=0;
	for(int loop=0;loop<numVars;loop++)
		if(var2Hes[loop].size()<=1) numSingleOccurrences++;
	//cout<<"Number of at most one occurrence vars: "<<numSingleOccurrences<<endl;

	vector<int> isChanged;	isChanged.resize(numVars);
	for(int i=0;i<numVars;i++) 	isChanged[i]=0;
	vector<int> oldFill;	oldFill.resize(numVars);
	vector<int> changed;
	long numTotalChanges=0;
	long numRequiredChanges=0;

	int elimWidth=0;
	int maxFill=numVars*numVars;
	int minFill=maxFill; int minVar;
	vector<vector<int> > var2Nei; var2Nei.resize(numVars); // var2Neighbors
	bool* isEliminated=new bool[numVars];

	//int* var2Fill=new int[numVars];
	vector<fillPair> var2FillPair; var2FillPair.resize(numVars);
	int MAXFILLSIZE=numVars*2;
    vector<vector<int> > fill2Vars; fill2Vars.resize(MAXFILLSIZE);

	for(int loop=0;loop<numVars;loop++) isEliminated[loop]=false;
	for(int loop=0;loop<numVars;loop++){
		for(vector<int>::iterator itr=var2Hes[loop].begin();itr!=var2Hes[loop].end();itr++)
			for(vector<int>::iterator itr2=hes[*itr].begin();itr2!=hes[*itr].end();itr2++)
				if(isPresent[*itr2]==false && loop!=*itr2){
					isPresent[*itr2]=true; var2Nei[loop].push_back(*itr2);
				}
		for(vector<int>::iterator itr=var2Nei[loop].begin();itr!=var2Nei[loop].end();itr++)
			isPresent[*itr]=false;
	}

	for(int loop=0;loop<numVars;loop++){
		int fill=(var2Nei[loop].size()*(var2Nei[loop].size()-1))/2; // verify this !!
		for(vector<int>::iterator itr=var2Nei[loop].begin();itr!=var2Nei[loop].end();itr++)
			isPresent[*itr]=true;
		for(vector<int>::iterator itr=var2Nei[loop].begin();itr!=var2Nei[loop].end();itr++){
			isPresent[*itr]=false;
			for(vector<int>::iterator itr2=var2Nei[*itr].begin();itr2!=var2Nei[*itr].end();itr2++)
				if(isPresent[*itr2]) fill--;
		}
		var2FillPair[loop].fill=fill;
		while((MAXFILLSIZE)<= fill){
			MAXFILLSIZE*=2; fill2Vars.resize(MAXFILLSIZE);
		}
		var2FillPair[loop].loc=fill2Vars[fill].size();
		fill2Vars[fill].push_back(loop);
		if(minFill>fill)	minFill=fill;
	}
	int numZeroFillVars=fill2Vars[0].size();
	//cout<<"Number of zero fill vars: "<<numZeroFillVars<<endl;

	double beginTime=cpuTime();

	vector<int>& vo=MinFillOrder;
    vo.clear();
	int startPos=0;

	for(int loop=0;loop<numZeroFillVars;loop++){
		int var=fill2Vars[0][loop];
		isEliminated[var]=true;
		vo.push_back(var);
		if(elimWidth<var2Nei[var].size()+1)
			elimWidth=var2Nei[var].size()+1;
	}
	fill2Vars[0].clear();
	startPos=numZeroFillVars;

	for(int loop=0;loop<numVars;loop++){
		for(vector<int>::iterator itr=var2Nei[loop].begin();itr!=var2Nei[loop].end();itr++){
			if(isEliminated[*itr]){
				*itr=var2Nei[loop][var2Nei[loop].size()-1];
				itr--;
				var2Nei[loop].pop_back();
			}
		}
	}

	while(fill2Vars[minFill].size()==0 && numZeroFillVars<numVars)
		minFill++;

	int numInternalZeroFills=0;

	for(int loopi=startPos;loopi<numVars;loopi++){
		//if(numZeroFillVars==loopi){
		//	cout<<"numZeroFillVars==loopi, time:  "<<cpuTime()-beginTime<<endl;
		//}
		//for(int loop2=0;loop2<numVars;loop2++)
		//	if(isEliminated[loop2]==false && minFill> var2Fill[loop2]){
		//		minFill=var2Fill[loop2]; minVar=loop2;
		//	}
		//cout<<"minFill: "<<minFill<<"   fill2Vars[minFill].size():"<<fill2Vars[minFill].size()<<endl;
		//assert(fill2Vars[minFill].size()!=0);
		minVar=fill2Vars[minFill][fill2Vars[minFill].size()-1];
		fill2Vars[minFill].pop_back();
		if(minFill==0) numInternalZeroFills++;
		while(fill2Vars[minFill].size()==0 && loopi<numVars-1)
			minFill++;
		vo.push_back(minVar);
		isEliminated[minVar]=true;
		int width=1; // to find the elimWidth we initialize to one
		for(vector<int>::iterator itr=var2Nei[minVar].begin();itr!=var2Nei[minVar].end();itr++)
			if(!isEliminated[*itr]) width++;
		if(width>elimWidth) elimWidth=width;
		for(vector<int>::iterator itr=var2Nei[minVar].begin();itr!=var2Nei[minVar].end();itr++){
			for(vector<int>::iterator itr2=var2Nei[*itr].begin();itr2!=var2Nei[*itr].end();itr2++)
				isPresent[*itr2]=true;
			for(vector<int>::iterator itr2=itr+1;itr2!=var2Nei[minVar].end();itr2++){
				if(!isPresent[*itr2]){
					// Add an edge and update the fill's
					var2Nei[*itr].push_back(*itr2);
					var2Nei[*itr2].push_back(*itr);
					for(vector<int>::iterator itr3=var2Nei[*itr2].begin();itr3!=var2Nei[*itr2].end();itr3++){
						if(isEliminated[*itr3]) continue;
						if(isPresent[*itr3]){
							if(isChanged[*itr3]==0){
								changed.push_back(*itr3);
								oldFill[*itr3]=var2FillPair[*itr3].fill;
							}
							isChanged[*itr3]++;
							var2FillPair[*itr3].fill--;
							/*
							int fillSize=var2FillPair[*itr3].fill-1;
							fill2Vars[var2FillPair[*itr3].fill][var2FillPair[*itr3].loc]=fill2Vars[var2FillPair[*itr3].fill][(fill2Vars[var2FillPair[*itr3].fill]).size()-1];
							var2FillPair[fill2Vars[var2FillPair[*itr3].fill][var2FillPair[*itr3].loc]].loc=var2FillPair[*itr3].loc;
							//assert(fill2Vars[var2FillPair[*itr3].fill].size()!=0);
							fill2Vars[var2FillPair[*itr3].fill].pop_back();
							var2FillPair[*itr3].fill=fillSize;
							var2FillPair[*itr3].loc=fill2Vars[fillSize].size();
							fill2Vars[fillSize].push_back(*itr3);
							if(minFill>fillSize)		minFill=fillSize;
							*/
						}
					}
				}
			}
			for(vector<int>::iterator itr2=var2Nei[*itr].begin();itr2!=var2Nei[*itr].end();itr2++)
				isPresent[*itr2]=false;
		}

		numRequiredChanges+=changed.size();
		for(vector<int>::iterator itr=changed.begin();itr!=changed.end();itr++){
			numTotalChanges+=isChanged[*itr];
			isChanged[*itr]=0;
			int cVar=*itr;//changedVar
			int fillSize=var2FillPair[cVar].fill;
			fill2Vars[oldFill[cVar]][var2FillPair[cVar].loc]=fill2Vars[oldFill[cVar]][(fill2Vars[oldFill[cVar]]).size()-1];
			var2FillPair[fill2Vars[oldFill[cVar]][var2FillPair[cVar].loc]].loc=var2FillPair[cVar].loc;
			//assert(fill2Vars[oldFill[cVar]].size()!=0);
			fill2Vars[oldFill[cVar]].pop_back();
			var2FillPair[cVar].loc=fill2Vars[fillSize].size();
			fill2Vars[fillSize].push_back(cVar);
			if(minFill>fillSize)		minFill=fillSize;

		}
		changed.clear();
		for(vector<int>::iterator itr=var2Nei[minVar].begin();itr!=var2Nei[minVar].end();itr++)
			for(vector<int>::iterator itr2=var2Nei[*itr].begin();itr2!=var2Nei[*itr].end();itr2++)
				if(isEliminated[*itr2]){
					*itr2=var2Nei[*itr][var2Nei[*itr].size()-1];
					itr2--; var2Nei[*itr].pop_back();
				}
		for(vector<int>::iterator itr=var2Nei[minVar].begin();itr!=var2Nei[minVar].end();itr++){
			int loop=*itr;
			int fill=(var2Nei[loop].size()*(var2Nei[loop].size()-1))/2; // verify this !!
			for(vector<int>::iterator itr=var2Nei[loop].begin();itr!=var2Nei[loop].end();itr++)
				isPresent[*itr]=true;
			for(vector<int>::iterator itr=var2Nei[loop].begin();itr!=var2Nei[loop].end();itr++){
				isPresent[*itr]=false;
				for(vector<int>::iterator itr2=var2Nei[*itr].begin();itr2!=var2Nei[*itr].end();itr2++)
					if(isPresent[*itr2]) fill--;
			}
			//*ATTN*
			fill2Vars[var2FillPair[loop].fill][var2FillPair[loop].loc]=fill2Vars[var2FillPair[loop].fill][(fill2Vars[var2FillPair[loop].fill]).size()-1];
			var2FillPair[fill2Vars[var2FillPair[loop].fill][var2FillPair[loop].loc]].loc=var2FillPair[loop].loc;
			//assert(fill2Vars[var2FillPair[loop].fill].size()!=0);
			fill2Vars[var2FillPair[loop].fill].pop_back();
			var2FillPair[loop].fill=fill;
			while((MAXFILLSIZE)<= fill){
				MAXFILLSIZE*=2; fill2Vars.resize(MAXFILLSIZE);
			}
			var2FillPair[loop].loc=fill2Vars[fill].size();
			//assert((fill2Vars.size()>fill));
			fill2Vars[fill].push_back(loop);
			if(minFill>fill)
				minFill=fill;
			while(fill2Vars[minFill].size()==0 && loopi<numVars-1)
				minFill++;
			//var2Fill[loop]=fill;

		}
	}
	//cout<<"Elimination width: "<<elimWidth<<endl;
	//cout<<"numTotalChanges: "<<numTotalChanges<<endl;
	//cout<<"numRequiredChanges: "<<numRequiredChanges<<endl;
	cout<<"numInternalZeroFills: "<<numInternalZeroFills<<endl;
	MinFillWidth=elimWidth-1;
}

void CMinFillTreeDecomposer::PrintMinFillOrder(){
	cout<<"MinFillOrder : ";
	for(int i=0,isize=MinFillOrder.size();i<isize;i++)
		cout<<MinFillOrder[i]<<" ";
	cout<<endl;
}

void CMinFillTreeDecomposer::Decompose(){
	// result: a tree decomposition corresponding to the MinFill order is obtained and stored in "ctg"
	ObtainMinFillOrder();
	ctd->SetElimOrder(MinFillOrder);
	ctd->ElimOrder2TreeDe();
	ctd->RIPCheck();
	if(MinFillWidth!=ctd->GetTreeWidth()){
		cout<<"MinFillWidth: "<<MinFillWidth<<"  ctd->GetTreeWidth(): "<< ctd->GetTreeWidth()<<endl;
		//assert(MinFillWidth==ctd->GetTreeWidth());
	}

}






