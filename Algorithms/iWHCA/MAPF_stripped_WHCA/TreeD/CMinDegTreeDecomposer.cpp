#include "CMinDegTreeDecomposer.h"

struct nhe{
	int size;	int *vars;
	int tNodesLoc;	int newHyperEdgesID;
};

struct degreePair{
	int degree;	int loc;
};

void CMinDegTreeDecomposer::ObtainMinDegOrder(){
	// read this function carefully before you compile
	int num_var=cchg->GetNumVars();
	int num_con=cchg->GetNumCons();
	vector<int>& vo=MinDegOrder;
	vector<vector<int> >& var2ches=cchg->GetVar2Hyperedges();
	vector<vector<int> >&  ches=cchg->GetHyperedges();

	int newHyperEdgesSequenceCounter=0;
	vector<nhe> newHyperEdges;
	vector<int> freeNewHyperEdgesLocs;
	vector<vector<int> > var2NewHyperEdges; var2NewHyperEdges.resize(num_var);
	vector<vector<int> > var2NewHyperEdgesID; var2NewHyperEdgesID.resize(num_var);
	vector<degreePair> var2DegreePair; var2DegreePair.resize(num_var);
	vector<vector<int> > degree2Vars; degree2Vars.resize(num_var);
	vector<bool> isPresent; for(int i=0;i<num_var;i++) isPresent.push_back(false);
	vector<bool> isPresent2; for(int i=0;i<num_var;i++) isPresent2.push_back(false);
	vector<bool> isDefunct; for(int i=0;i<num_con;i++) isDefunct.push_back(false);
	vector<int> presents;
	vector<int> presents2;
	vector<int> var2NumOccurences; var2NumOccurences.resize(num_var);
	for(int i=0;i<num_var;i++)	var2NumOccurences[i]=0;
	for(int i=0;i<num_con;i++)
		for(vector<int>::iterator itr=ches[i].begin();itr!=ches[i].end();itr++)
			var2NumOccurences[*itr]++;
	int elimWidth=0;
	vector<int>  decreaseCount; decreaseCount.resize(num_var); 
	for(int i=0;i<num_var;i++) decreaseCount[i]=0;
	int minDegree=num_var;
	for(int i=0;i<num_var;i++)
		var2DegreePair[i].degree=0;
	for(int i=0;i<num_var-1;i++){
		for(vector<int>::iterator itr=var2ches[i].begin();itr!=var2ches[i].end();itr++){
			for(vector<int>::iterator itr2=ches[*itr].begin();itr2!=ches[*itr].end();itr2++){
				if(!isPresent[*itr2] && *itr2>i){
					presents.push_back(*itr2);
					isPresent[*itr2]=true;
				}
			}
		}
		var2DegreePair[i].degree+=presents.size();
		for(vector<int>::iterator itr=presents.begin();itr!=presents.end();itr++){
			isPresent[*itr]=false;
			var2DegreePair[*itr].degree++;
		}
		presents.clear();
	}


	for(int i=0;i<num_var;i++){
		if(minDegree>var2DegreePair[i].degree)
			minDegree=var2DegreePair[i].degree;
		var2DegreePair[i].loc=degree2Vars[var2DegreePair[i].degree].size();
		degree2Vars[var2DegreePair[i].degree].push_back(i);
	}
	
	vo.resize(num_var);
	for(int i=0;i<num_var;i++){
		int currVar=degree2Vars[minDegree][degree2Vars[minDegree].size()-1];
		degree2Vars[minDegree].pop_back();
		while(degree2Vars[minDegree].size()==0 && i<num_var-1)
			minDegree++;
		vo[i]=currVar;
		if(var2NumOccurences[currVar]==0)
			continue;
		for(vector<int>::iterator itr=var2ches[currVar].begin();itr!=var2ches[currVar].end();itr++){
			int currCon=*itr;
			if(isDefunct[currCon]) continue;
			isDefunct[currCon]=true;
			for(vector<int>::iterator itr2=ches[currCon].begin();itr2!=ches[currCon].end();itr2++){
					if(!isPresent[*itr2]){
						presents.push_back(*itr2);
						isPresent[*itr2]=true;
					}
					else
						decreaseCount[*itr2]++;
			}
			if(elimWidth<ches[currCon].size())
				elimWidth=ches[currCon].size();
		}
		int idItr=0;
		for(vector<int>::iterator itr=var2NewHyperEdges[currVar].begin();itr!=var2NewHyperEdges[currVar].end();itr++,idItr++){
			nhe nh=newHyperEdges[*itr];
			if(nh.newHyperEdgesID!=var2NewHyperEdgesID[currVar][idItr]) continue;
			for(int itr2=0;itr2<nh.size;itr2++){
					if(!isPresent[nh.vars[itr2]]){
						presents.push_back(nh.vars[itr2]);
						isPresent[nh.vars[itr2]]=true;
					}
					else
						decreaseCount[nh.vars[itr2]]++;
			}
			delete nh.vars;
		}

		if(elimWidth<presents.size())
			elimWidth=presents.size();

		idItr=0;
		for(vector<int>::iterator itr=var2NewHyperEdges[currVar].begin();itr!=var2NewHyperEdges[currVar].end();itr++,idItr++){

			nhe nh=newHyperEdges[*itr];
			if(nh.newHyperEdgesID!=var2NewHyperEdgesID[currVar][idItr]) continue;
			
			newHyperEdges[*itr].newHyperEdgesID=-1;
			freeNewHyperEdgesLocs.push_back(*itr);
		}
		
		for(vector<int>::iterator itr=presents.begin();itr!=presents.end();itr++){
			var2NumOccurences[*itr]-=decreaseCount[*itr];
			decreaseCount[*itr]=0;
			if(var2NumOccurences[*itr]==1){
				var2NumOccurences[*itr]=0;
				if(*itr!=currVar){
					i++;
					vo[i]=*itr;
					degree2Vars[var2DegreePair[*itr].degree][var2DegreePair[*itr].loc]=degree2Vars[var2DegreePair[*itr].degree][degree2Vars[var2DegreePair[*itr].degree].size()-1];
					var2DegreePair[degree2Vars[var2DegreePair[*itr].degree][var2DegreePair[*itr].loc]].loc=var2DegreePair[*itr].loc;
					degree2Vars[var2DegreePair[*itr].degree].pop_back();
				}
				*itr=presents[presents.size()-1];
				presents.pop_back();
				itr--;
				continue;
			}
			isPresent[*itr]=false;
		}

		while(degree2Vars[minDegree].size()==0 && i<num_var-1)
			minDegree++;

		if(presents.size()==0) continue;
		nhe tempnh;
		tempnh.size=presents.size();
		tempnh.vars=new int[tempnh.size];
		for(int tempi=0;tempi<tempnh.size;tempi++)
			tempnh.vars[tempi]=presents[tempi];
		int newLoc;
		tempnh.newHyperEdgesID=newHyperEdgesSequenceCounter;
		if(freeNewHyperEdgesLocs.size()!=0){
			newLoc=freeNewHyperEdgesLocs[freeNewHyperEdgesLocs.size()-1];
			freeNewHyperEdgesLocs.pop_back();
		}
		else{
			newLoc=newHyperEdges.size();
			newHyperEdges.resize(newHyperEdges.size()+1);
		}
		newHyperEdges[newLoc]=tempnh;
		for(vector<int>::iterator itr=presents.begin();itr!=presents.end();itr++){
			var2NewHyperEdges[*itr].push_back(newLoc);
			var2NewHyperEdgesID[*itr].push_back(newHyperEdgesSequenceCounter);
		}
		newHyperEdgesSequenceCounter++;
		for(vector<int>::iterator itr=presents.begin();itr!=presents.end();itr++){
			for(vector<int>::iterator itr2=var2ches[*itr].begin();itr2!=var2ches[*itr].end();itr2++){
				if(isDefunct[*itr2]) continue;
				for(vector<int>::iterator itr3=ches[*itr2].begin();itr3!=ches[*itr2].end();itr3++){
					if(!isPresent2[*itr3]){
						isPresent2[*itr3]=true;
						presents2.push_back(*itr3);
					}
				}
			}
			idItr=0;
			for(vector<int>::iterator itr2=var2NewHyperEdges[*itr].begin();itr2!=var2NewHyperEdges[*itr].end();itr2++,idItr++){
				if(newHyperEdges[*itr2].newHyperEdgesID==var2NewHyperEdgesID[*itr][idItr]){
					nhe nh=newHyperEdges[*itr2];
					for(int k=0;k<nh.size;k++){
						if(!isPresent2[nh.vars[k]]){
							isPresent2[nh.vars[k]]=true;
							presents2.push_back(nh.vars[k]);
						}
					}
					
				}
			}
			degree2Vars[var2DegreePair[*itr].degree][var2DegreePair[*itr].loc]=degree2Vars[var2DegreePair[*itr].degree][(degree2Vars[var2DegreePair[*itr].degree]).size()-1];
			var2DegreePair[degree2Vars[var2DegreePair[*itr].degree][var2DegreePair[*itr].loc]].loc=var2DegreePair[*itr].loc;
			degree2Vars[var2DegreePair[*itr].degree].pop_back();
			var2DegreePair[*itr].degree=presents2.size();
			var2DegreePair[*itr].loc=degree2Vars[presents2.size()].size();
			degree2Vars[presents2.size()].push_back(*itr);
			if(minDegree>presents2.size())
				minDegree=presents2.size();
			while(degree2Vars[minDegree].size()==0 && i<num_var-1)
				minDegree++;
			for(vector<int>::iterator itr2=presents2.begin();itr2!=presents2.end();itr2++)
				isPresent2[*itr2]=false;
			presents2.clear();
		}
		presents.clear();
	}
	//cout<<"Elimination width: "<<elimWidth<<endl; 
	MinDegWidth=elimWidth-1;
}

void CMinDegTreeDecomposer::PrintMinDegOrder(){
	cout<<"MinDegOrder : ";
	for(int i=0,isize=MinDegOrder.size();i<isize;i++)
		cout<<MinDegOrder[i]<<" ";
	cout<<endl;
}

void CMinDegTreeDecomposer::Decompose(){
	// result: a tree decomposition corresponding to the MinDeg order is obtained and stored in "ctg" 
	ObtainMinDegOrder();
	ctd->SetElimOrder(MinDegOrder);
	ctd->ElimOrder2TreeDe();
	ctd->RIPCheck();
	assert(MinDegWidth==ctd->GetTreeWidth());
}






