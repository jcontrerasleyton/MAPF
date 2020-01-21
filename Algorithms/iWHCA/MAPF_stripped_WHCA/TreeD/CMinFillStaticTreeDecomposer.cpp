#include "CMinFillStaticTreeDecomposer.h"

void CMinFillStaticTreeDecomposer::ObtainMinFillStaticOrder(){
	// read this function carefully before you compile
	int numVars=cchg->GetNumVars();
	vector<vector<int> >& var2Hes=cchg->GetVar2Hyperedges();
	vector<vector<int> >&  hes=cchg->GetHyperedges();

	vector<bool> isPresent;	isPresent.resize(numVars);
	for(int i=0;i<numVars;i++) 	isPresent[i]=false;

	int elimWidth=0;
	int maxFill=numVars*numVars;

	vector<vector<int> > var2Nei; var2Nei.resize(numVars); // var2Neighbors
	bool* isEliminated=new bool[numVars];
	int* var2Fill=new int[numVars];
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
		var2Fill[loop]=fill;
	}

	vector<int>& vo=MinFillStaticOrder;
    vo.clear();
	for(int loop=0;loop<numVars;loop++){
		int minFill=maxFill; int minVar;
		for(int loop2=0;loop2<numVars;loop2++)
			if(isEliminated[loop2]==false && minFill> var2Fill[loop2]){
				minFill=var2Fill[loop2]; minVar=loop2;
			}
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
					for(vector<int>::iterator itr3=var2Nei[*itr2].begin();itr3!=var2Nei[*itr2].end();itr3++)
						if(isPresent[*itr3]) var2Fill[*itr3]--;
				}
			}
			for(vector<int>::iterator itr2=var2Nei[*itr].begin();itr2!=var2Nei[*itr].end();itr2++)
				isPresent[*itr2]=false;
		}
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
			var2Fill[loop]=fill;
		}
	}
	//cout<<"Elimination width: "<<elimWidth<<endl;
	MinFillStaticWidth=elimWidth-1;
}

void CMinFillStaticTreeDecomposer::PrintMinFillStaticOrder(){
	cout<<"MinFillStaticOrder : ";
	for(int i=0,isize=MinFillStaticOrder.size();i<isize;i++)
		cout<<MinFillStaticOrder[i]<<" ";
	cout<<endl;
}

void CMinFillStaticTreeDecomposer::Decompose(){
	// result: a tree decomposition corresponding to the MinFillStatic order is obtained and stored in "ctg"
	ObtainMinFillStaticOrder();
	ctd->SetElimOrder(MinFillStaticOrder);
	ctd->ElimOrder2TreeDe();
	ctd->RIPCheck();
	if(MinFillStaticWidth!=ctd->GetTreeWidth()){
		cout<<"MinFillStaticWidth: "<<MinFillStaticWidth<<"  ctd->GetTreeWidth(): "<< ctd->GetTreeWidth()<<endl;
		assert(MinFillStaticWidth==ctd->GetTreeWidth());
	}

}






