#include "CMinFillDegTreeDecomposer.h"

struct nhe{
	int size;	int *vars;
	int tNodesLoc;	int newHyperEdgesID;
};

struct fillDegPair{
	int fill; int degree;	int loc;
};

void CMinFillDegTreeDecomposer::ObtainMinFillDegOrder(){
	// read this function carefully before you compile
	int num_var=cchg->GetNumVars();
	int num_con=cchg->GetNumCons();
	vector<int>& vo=MinFillDegOrder;
	vector<vector<int> >& var2ches=cchg->GetVar2Hyperedges();
	vector<vector<int> >&  ches=cchg->GetHyperedges();

	int newHyperEdgesSequenceCounter=0;
	vector<nhe> newHyperEdges;
	vector<int> freeNewHyperEdgesLocs;
	vector<vector<int> > var2NewHyperEdges; var2NewHyperEdges.resize(num_var);
	vector<vector<int> > var2NewHyperEdgesID; var2NewHyperEdgesID.resize(num_var);
	vector<fillDegPair> var2FillDegPair; var2FillDegPair.resize(num_var);
	int MAXFILLSIZE=num_var*2;
	vector<vector<int> > fillDeg2Vars; fillDeg2Vars.resize(MAXFILLSIZE);
	vector<bool> isPresent; for(int i=0;i<num_var;i++) isPresent.push_back(false);
	vector<bool> isFillPresent; for(int i=0;i<num_var;i++) isFillPresent.push_back(false);
	vector<bool> isPresent2; for(int i=0;i<num_var;i++) isPresent2.push_back(false);
	vector<bool> isDefunct; for(int i=0;i<num_con;i++) isDefunct.push_back(false);
	vector<int> presents;
	vector<int> fillPresents;
	vector<int> presents2;
	vector<int> var2NumOccurences; var2NumOccurences.resize(num_var);
	for(int i=0;i<num_var;i++)	var2NumOccurences[i]=0;
	for(int i=0;i<num_con;i++)
		for(vector<int>::iterator itr=ches[i].begin();itr!=ches[i].end();itr++)
			var2NumOccurences[*itr]++;
	int elimWidth=0;
	vector<int>  decreaseCount; decreaseCount.resize(num_var);
	for(int i=0;i<num_var;i++) decreaseCount[i]=0;
	int minFill=num_var*num_var;
	for(int i=0;i<num_var;i++)
		var2FillDegPair[i].fill=0;
	for(int i=0;i<num_var;i++){
		for(vector<int>::iterator itr=var2ches[i].begin();itr!=var2ches[i].end();itr++){
			for(vector<int>::iterator itr2=ches[*itr].begin();itr2!=ches[*itr].end();itr2++){
				if(!isPresent[*itr2]){
					presents.push_back(*itr2);
					isPresent[*itr2]=true;
				}
			}
		}
		//-----------FILL-CALC----------------
		int fillSize=presents.size()*presents.size();
		for(int loop=0;loop<presents.size();loop++){
			int fVar=presents[loop];
			for(vector<int>::iterator fitr=var2ches[fVar].begin();fitr!=var2ches[fVar].end();fitr++){
				int fCon=*fitr;
				for(vector<int>::iterator fitr2=ches[fCon].begin();fitr2!=ches[fCon].end();fitr2++){
					if(!isFillPresent[*fitr2]){
						isFillPresent[*fitr2]=true;
						fillPresents.push_back(*fitr2);
						if(isPresent[*fitr2]) fillSize--;
					}
				}
			}
			for(vector<int>::iterator fitr=fillPresents.begin();fitr!=fillPresents.end();fitr++)
				isFillPresent[*fitr]=false;
			fillPresents.clear();
		}
		//------------------------------------
		var2FillDegPair[i].fill=fillSize;
		var2FillDegPair[i].degree=presents.size();
		for(vector<int>::iterator itr=presents.begin();itr!=presents.end();itr++)
			isPresent[*itr]=false;
		presents.clear();
	}


	for(int i=0;i<num_var;i++){
		if(minFill>var2FillDegPair[i].fill)
			minFill=var2FillDegPair[i].fill;
		while((MAXFILLSIZE+1)<= var2FillDegPair[i].fill){
			MAXFILLSIZE*=2; fillDeg2Vars.resize(MAXFILLSIZE);
		}
		var2FillDegPair[i].loc=fillDeg2Vars[var2FillDegPair[i].fill].size();
		fillDeg2Vars[var2FillDegPair[i].fill].push_back(i);
	}

	vo.resize(num_var);
	for(int i=0;i<num_var;i++){

		cerr<<"   "<<fillDeg2Vars[minFill].size()<<"-"<<minFill;
		for(int checkloop=0;checkloop<fillDeg2Vars[minFill].size();checkloop++)
			cerr<<" "<<var2FillDegPair[fillDeg2Vars[minFill][checkloop]].degree;
		cerr<<endl;

		int minDegPos=0; int minDeg=var2FillDegPair[fillDeg2Vars[minFill][0]].degree;
		for(int checkloop=0;checkloop<fillDeg2Vars[minFill].size();checkloop++)
			if(var2FillDegPair[fillDeg2Vars[minFill][checkloop]].degree<minDeg){
				minDeg=var2FillDegPair[fillDeg2Vars[minFill][checkloop]].degree;
				minDegPos=checkloop;
			}
			
		int currVar=fillDeg2Vars[minFill][minDegPos];

		fillDeg2Vars[minFill][minDegPos]=fillDeg2Vars[minFill][fillDeg2Vars[minFill].size()-1];
		var2FillDegPair[fillDeg2Vars[minFill][minDegPos]].loc=minDegPos;
		fillDeg2Vars[minFill].pop_back();


		while(fillDeg2Vars[minFill].size()==0 && i<num_var-1)
			minFill++;
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
					fillDeg2Vars[var2FillDegPair[*itr].fill][var2FillDegPair[*itr].loc]=fillDeg2Vars[var2FillDegPair[*itr].fill][fillDeg2Vars[var2FillDegPair[*itr].fill].size()-1];
					var2FillDegPair[fillDeg2Vars[var2FillDegPair[*itr].fill][var2FillDegPair[*itr].loc]].loc=var2FillDegPair[*itr].loc;
					fillDeg2Vars[var2FillDegPair[*itr].fill].pop_back();
				}
				*itr=presents[presents.size()-1];
				presents.pop_back();
				itr--;
				continue;
			}
			isPresent[*itr]=false;
		}

		while(fillDeg2Vars[minFill].size()==0 && i<num_var-1)
			minFill++;

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


			//-----------FILL-CALC----------------
			int fillSize=presents2.size()*presents2.size();
			for(int loop=0;loop<presents2.size();loop++){
				int fVar=presents2[loop];
				for(vector<int>::iterator fitr=var2ches[fVar].begin();fitr!=var2ches[fVar].end();fitr++){
					int fCon=*fitr;
					if(isDefunct[fCon]) continue;
					for(vector<int>::iterator fitr2=ches[fCon].begin();fitr2!=ches[fCon].end();fitr2++){
						if(!isFillPresent[*fitr2]){
							isFillPresent[*fitr2]=true;
							fillPresents.push_back(*fitr2);
							if(isPresent2[*fitr2]) fillSize--;
						}
					}
				}

				int idItr=0;
				for(vector<int>::iterator fitr=var2NewHyperEdges[fVar].begin();fitr!=var2NewHyperEdges[fVar].end();fitr++,idItr++){
					nhe nh=newHyperEdges[*fitr];
					if(nh.newHyperEdgesID!=var2NewHyperEdgesID[fVar][idItr]) continue;
					for(int fitr2=0;fitr2<nh.size;fitr2++){
							if(!isFillPresent[nh.vars[fitr2]]){
								fillPresents.push_back(nh.vars[fitr2]);
								isFillPresent[nh.vars[fitr2]]=true;
								if(isPresent2[nh.vars[fitr2]]) fillSize--;
							}
					}
				}

				for(vector<int>::iterator fitr=fillPresents.begin();fitr!=fillPresents.end();fitr++)
					isFillPresent[*fitr]=false;
				var2FillDegPair[fVar].degree=fillPresents.size();
				fillPresents.clear();
			}
			//------------------------------------

			fillDeg2Vars[var2FillDegPair[*itr].fill][var2FillDegPair[*itr].loc]=fillDeg2Vars[var2FillDegPair[*itr].fill][(fillDeg2Vars[var2FillDegPair[*itr].fill]).size()-1];
			var2FillDegPair[fillDeg2Vars[var2FillDegPair[*itr].fill][var2FillDegPair[*itr].loc]].loc=var2FillDegPair[*itr].loc;
			fillDeg2Vars[var2FillDegPair[*itr].fill].pop_back();
			var2FillDegPair[*itr].fill=fillSize;
			//var2FillDegPair[*itr].degree=presents2.size();
			while((MAXFILLSIZE+1)<= fillSize){
				MAXFILLSIZE*=2; fillDeg2Vars.resize(MAXFILLSIZE);
			}
			var2FillDegPair[*itr].loc=fillDeg2Vars[fillSize].size();
			fillDeg2Vars[fillSize].push_back(*itr);
			if(minFill>fillSize)
				minFill=fillSize;
			while(fillDeg2Vars[minFill].size()==0 && i<num_var-1)
				minFill++;
			for(vector<int>::iterator itr2=presents2.begin();itr2!=presents2.end();itr2++)
				isPresent2[*itr2]=false;
			presents2.clear();
		}
		presents.clear();
	}
	//cout<<"Elimination width: "<<elimWidth<<endl;
	MinFillDegWidth=elimWidth-1;
}

void CMinFillDegTreeDecomposer::PrintMinFillDegOrder(){
	cout<<"MinFillDegOrder : ";
	for(int i=0,isize=MinFillDegOrder.size();i<isize;i++)
		cout<<MinFillDegOrder[i]<<" ";
	cout<<endl;
}

void CMinFillDegTreeDecomposer::Decompose(){
	// result: a tree decomposition corresponding to the MinFillDeg order is obtained and stored in "ctg"
	ObtainMinFillDegOrder();
	ctd->SetElimOrder(MinFillDegOrder);
	ctd->ElimOrder2TreeDe();
	ctd->RIPCheck();
	assert(MinFillDegWidth==ctd->GetTreeWidth());
}






