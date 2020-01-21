#include "CMCSTreeDecomposer.h"

void CMCSTreeDecomposer::ObtainMCSOrder(){
	// result: the MCSWidth and MCSOrder members are updated

	int maxCount=0;
	MCSWidth=0;

	int num_var=cchg->GetNumVars();
	vector<int>& vo=MCSOrder; 
	vector<vector<int> >& var2ches=cchg->GetVar2Hyperedges();

	vector<countPair> var2countPair; var2countPair.resize(num_var);
	vector<vector<int> > count2vars; count2vars.resize(1);
	for(int i=0;i<num_var;i++){
		var2countPair[i].count=0;
		var2countPair[i].loc=count2vars[0].size();
		count2vars[0].push_back(i);
	}

	vector<int> neighbors;
	vector<bool> isMarkedNeighbor;  for(int i=0;i<num_var;i++)	isMarkedNeighbor.push_back(false);
	vo.resize(num_var);
	for(int i=0;i<num_var;i++){
		int currVar=count2vars[maxCount][count2vars[maxCount].size()-1];
		vo[num_var-1-i]=currVar;
		count2vars[maxCount].pop_back();
		if(MCSWidth<maxCount)
			MCSWidth=maxCount;
		while(count2vars[maxCount].size()==0)
			maxCount--;
		isMarkedNeighbor[currVar]=true;
		for(vector<int>::iterator itr=var2ches[currVar].begin();itr!=var2ches[currVar].end();itr++){
			int currCon=*itr;
			vector<int>& hyperedge=cchg->GetHyperegde(currCon);
			for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++){
				int currNeighbor=*itr2;
				if(!isMarkedNeighbor[currNeighbor]){
					neighbors.push_back(currNeighbor);
					isMarkedNeighbor[currNeighbor]=true;
				}
			}
		}
		for(vector<int>::iterator itr=neighbors.begin();itr!=neighbors.end();itr++){
			int oldLoc=var2countPair[*itr].loc;
			int neighCount=var2countPair[*itr].count;
			count2vars[neighCount][oldLoc]=count2vars[neighCount][count2vars[neighCount].size()-1];
			var2countPair[count2vars[neighCount][oldLoc]].loc=oldLoc;
			count2vars[neighCount].pop_back();
			neighCount++;
			if(count2vars.size()==neighCount)
				count2vars.resize(count2vars.size()+1);
			var2countPair[*itr].loc=count2vars[neighCount].size();
			var2countPair[*itr].count=neighCount;
			count2vars[neighCount].push_back(*itr);
			isMarkedNeighbor[*itr]=false;
			if(maxCount<neighCount)
				maxCount=neighCount;
		}
		neighbors.clear();
	}
	
}

void CMCSTreeDecomposer::PrintMCSOrder(){
	cout<<"MCSOrder : ";
	for(int i=0,isize=MCSOrder.size();i<isize;i++)
		cout<<MCSOrder[i]<<" ";
	cout<<endl;
}

void CMCSTreeDecomposer::Decompose(){
	// result: a tree decomposition corresponding to the MCS order is obtained and stored in "ctg" 
	ObtainMCSOrder();
	ctd->SetElimOrder(MCSOrder);
	ctd->ElimOrder2TreeDe();
	ctd->RIPCheck();
}
