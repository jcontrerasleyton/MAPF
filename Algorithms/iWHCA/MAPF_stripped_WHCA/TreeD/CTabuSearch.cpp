#include "CTabuSearch.h"
#include "cputime.h"

CTabuSearch::CTabuSearch(CTreeDecomposition* cctd){
	ctd=cctd;
	cchg=ctd->GetConstraintHypergraph();
	vector<STreeNode>& sTreeNodes=ctd->GetTreeNodes();
	sTabuTreeNodes.resize(sTreeNodes.size());
	numVars=cchg->GetNumVars();
	numCons=cchg->GetNumCons();
	var2Loc.resize(numVars);
	isPresent.resize(numVars);	for(int i=0;i<numVars;i++)	isPresent[i]=false;
	isEliminated.resize(numVars);	for(int i=0;i<numVars;i++)	isEliminated[i]=false;
	isCovered.resize(numCons);	for(int i=0;i<numCons;i++)	isCovered[i]=false;

	//Mega Copy
	root=-1;
	for(int i=0,isize=sTreeNodes.size();i<isize;i++){
		STreeNode& stn=sTreeNodes[i];
		STabuTreeNode& sttn=sTabuTreeNodes[i];
		sttn.childNodes=stn.childNodes;
		sttn.coveredCons=stn.coveredCons;
		sttn.elimVar=stn.elimVar;
		var2Loc[sttn.elimVar]=i;
		sttn.otherVars=stn.otherVars;
		sttn.parent=stn.parent;
		if(stn.parent==-1){
			if(root==-1){
				root=i;
			}
			else{
				cout<<"Error \"CTabuSearch\": Unexpected case, more than one root in the tree decomposition."<<endl;
				cout<<"Current root value "<<root<<endl;
				exit(1);
			}
		}
	}
	sTabuTreeNodes[root].parentalWidth=-1;
	FindSubTreeValues(root);
	sTabuTreeNodes[root].parentalQuadSum=0;
	UpdateParentalValues(root);

	bestWidth=sTabuTreeNodes[root].mySubTreeWidth;
	//cout<<"bestWidth "<<bestWidth<<endl;
}

void CTabuSearch::Print(){
	cout<<"Printing the TabuSearchTree ..."<<endl;
	for(int i=0,isize=sTabuTreeNodes.size();i<isize;i++){
		STabuTreeNode& sttn=sTabuTreeNodes[i];
		cout<<"Node "<<i<<" :"<<endl;
		cout<<"\t"<<"parent  "<<sttn.parent<<endl;
		cout<<"\t"<<"Vars : ";
		cout<<sttn.elimVar<<" ";
		for(vector<int>::iterator itr=sttn.otherVars.begin();itr!=sttn.otherVars.end();itr++)
			cout<<*itr<<" ";
		cout<<endl;
		cout<<"\t"<<"CoveredCons : ";
		for(vector<int>::iterator itr=sttn.coveredCons.begin();itr!=sttn.coveredCons.end();itr++)
			cout<<*itr<<" ";
		cout<<endl;
		cout<<"\t"<<"ChildNodes : ";
		for(vector<int>::iterator itr=sttn.childNodes.begin();itr!=sttn.childNodes.end();itr++)
			cout<<*itr<<" ";
		cout<<endl;
		cout<<"\t"<<"mySTW  "<<sttn.mySubTreeWidth  <<endl;
		cout<<"\t"<<"mySTQS  "<<sttn.mySubTreeQuadSum  <<endl;
		cout<<"\t"<<"parenW  "<<sttn.parentalWidth  <<endl;
		cout<<"\t"<<"parenQS  "<<sttn.parentalQuadSum  <<endl;
		cout<<endl;
	}

}

void CTabuSearch::FindSubTreeValues(int ptr){
	STabuTreeNode& sttn=sTabuTreeNodes[ptr];
	for(int i=0,isize=sttn.childNodes.size();i<isize;i++)
		FindSubTreeValues(sttn.childNodes[i]);
	int maxWidth=sttn.otherVars.size();
	for(int i=0,isize=sttn.childNodes.size();i<isize;i++)
		if(maxWidth<sTabuTreeNodes[sttn.childNodes[i]].mySubTreeWidth)
			maxWidth=sTabuTreeNodes[sttn.childNodes[i]].mySubTreeWidth;
	sttn.mySubTreeWidth=maxWidth;
	long quadSum=sttn.otherVars.size()*sttn.otherVars.size();
	for(int i=0,isize=sttn.childNodes.size();i<isize;i++)
		quadSum+=sTabuTreeNodes[sttn.childNodes[i]].mySubTreeQuadSum;
	sttn.mySubTreeQuadSum=quadSum;
}

void CTabuSearch::UpdateParentalValues(int ptr){
	STabuTreeNode& sttn=sTabuTreeNodes[ptr];
	int otherVarsSize=sttn.otherVars.size();
	int childNodesSize=sttn.childNodes.size();

	if(childNodesSize==0) return; // leaf case 
	
	vector<int> ownValue; ownValue.resize(childNodesSize); 
	vector<int> up; up.resize(childNodesSize);
	vector<int> down; down.resize(childNodesSize);
	
	for(int i=0;i<childNodesSize;i++)
		ownValue[i]=sTabuTreeNodes[sttn.childNodes[i]].mySubTreeWidth;

	if(sttn.parentalWidth<otherVarsSize) 
		up[0]=otherVarsSize;
	else
		up[0]=sttn.parentalWidth;
	for(int i=1;i<childNodesSize;i++)
		if(ownValue[i-1]<up[i-1])
			up[i]=up[i-1];
		else
			up[i]=ownValue[i-1];
	
	if(sttn.parentalWidth<otherVarsSize) 
		down[childNodesSize-1]=otherVarsSize;
	else
		down[childNodesSize-1]=sttn.parentalWidth;
	for(int i=childNodesSize-2;i>=0;i--)
		if(ownValue[i+1]<down[i+1])
			down[i]=down[i+1];
		else
			down[i]=ownValue[i+1];

	long localSum=sttn.parentalQuadSum+sttn.mySubTreeQuadSum;	
	for(int i=0;i<childNodesSize;i++){
		if(up[i]>down[i])
			sTabuTreeNodes[sttn.childNodes[i]].parentalWidth=up[i];
		else
			sTabuTreeNodes[sttn.childNodes[i]].parentalWidth=down[i];		
		sTabuTreeNodes[sttn.childNodes[i]].parentalQuadSum = localSum
															-sTabuTreeNodes[sttn.childNodes[i]].mySubTreeQuadSum;
	}

	for(int i=0;i<childNodesSize;i++)
		UpdateParentalValues(sttn.childNodes[i]);

}

SWidthSumPair CTabuSearch::Evaluate(int var){
	//var is the variable whose up-move you wish to evaluate
	SWidthSumPair sWidthSumPair;
	STabuTreeNode& sttnB=sTabuTreeNodes[var2Loc[var]];
	if(sttnB.parent==-1){
		sWidthSumPair.width=sttnB.mySubTreeWidth;
		sWidthSumPair.sum=sttnB.mySubTreeQuadSum;
		return sWidthSumPair;
	}

	STabuTreeNode& sttnA=sTabuTreeNodes[sttnB.parent];

	bool inA;
	int elimVarA,elimVarB;
	vector<int> newACoveredCons,newAChildNodes,newAOtherVars;
	vector<int> newBCoveredCons,newBChildNodes,newBOtherVars;
	newACoveredCons=sttnA.coveredCons;
	newAChildNodes=sttnA.childNodes; // remember 'B' needs to be removed from this vector

	elimVarA=sttnA.elimVar; elimVarB=sttnB.elimVar;

	for(vector<int>::iterator itr=sttnB.coveredCons.begin();itr!=sttnB.coveredCons.end();itr++){
		vector<int>& hyperedge=cchg->GetHyperegde(*itr);
		inA=false;
		for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++)
			if(*itr2==elimVarA){
				inA=true; break;
			}

		if(inA)		newACoveredCons.push_back(*itr);
		else		newBCoveredCons.push_back(*itr);
	}
	
	for(vector<int>::iterator itr=sttnB.childNodes.begin();itr!=sttnB.childNodes.end();itr++){
		vector<int>& otherVars=sTabuTreeNodes[*itr].otherVars;
		inA=false;
		for(vector<int>::iterator itr2=otherVars.begin();itr2!=otherVars.end();itr2++)
			if(*itr2==elimVarA){
				inA=true; break;
			}

		if(inA)	newAChildNodes.push_back(*itr);
		else 	newBChildNodes.push_back(*itr);
	}
	
	//Note: there is a minor violation, since newBChildNodes has to contain newA-node
	
	int newTreeWidth=-1; // to store the treeWidth of the move-up resulting decomposition

	//code to obtain newAOtherVars, 'remember that the old-B is not a child node of newA
	for(vector<int>::iterator itr=newACoveredCons.begin();itr!=newACoveredCons.end();itr++){
		vector<int>& hyperedge=cchg->GetHyperegde(*itr);
		for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++)
			if(*itr2!=elimVarA && !isPresent[*itr2]){
				isPresent[*itr2]=true; newAOtherVars.push_back(*itr2);
			}
	}
	for(vector<int>::iterator itr=newAChildNodes.begin();itr!=newAChildNodes.end();itr++){
		if(*itr==var2Loc[var]) continue; // skipping the old-B, its not a child node of newA
		vector<int>& otherVars=sTabuTreeNodes[*itr].otherVars;
		if(sTabuTreeNodes[*itr].mySubTreeWidth>newTreeWidth)
			newTreeWidth=sTabuTreeNodes[*itr].mySubTreeWidth;
		for(vector<int>::iterator itr2=otherVars.begin();itr2!=otherVars.end();itr2++)
			if(!isPresent[*itr2] && *itr2!=elimVarA){
				isPresent[*itr2]=true; newAOtherVars.push_back(*itr2);
			}
	}

	for(int i=0,isize=newAOtherVars.size();i<isize;i++)
		isPresent[newAOtherVars[i]]=false;

	if(newTreeWidth<newAOtherVars.size())	newTreeWidth=newAOtherVars.size();

	for(vector<int>::iterator itr=newBCoveredCons.begin();itr!=newBCoveredCons.end();itr++){
		vector<int>& hyperedge=cchg->GetHyperegde(*itr);
		for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++)
			if(*itr2!=elimVarB && !isPresent[*itr2]){
				isPresent[*itr2]=true; newBOtherVars.push_back(*itr2);
			}
	}

	for(vector<int>::iterator itr=newBChildNodes.begin();itr!=newBChildNodes.end();itr++){
		vector<int>& otherVars=sTabuTreeNodes[*itr].otherVars;
		if(sTabuTreeNodes[*itr].mySubTreeWidth>newTreeWidth)
			newTreeWidth=sTabuTreeNodes[*itr].mySubTreeWidth;
		for(vector<int>::iterator itr2=otherVars.begin();itr2!=otherVars.end();itr2++)
			if(!isPresent[*itr2] && *itr2!=elimVarB){
				isPresent[*itr2]=true; newBOtherVars.push_back(*itr2);
			}
	}
	
	for(vector<int>::iterator itr2=newAOtherVars.begin();itr2!=newAOtherVars.end();itr2++)
		if(!isPresent[*itr2] && *itr2!=elimVarB){
			isPresent[*itr2]=true; newBOtherVars.push_back(*itr2);
		}

	for(int i=0,isize=newBOtherVars.size();i<isize;i++)
		isPresent[newBOtherVars[i]]=false;

	if(newTreeWidth<newBOtherVars.size()) newTreeWidth=newBOtherVars.size();
	if(newTreeWidth<sttnA.parentalWidth)	newTreeWidth=sttnA.parentalWidth;
	sWidthSumPair.width=newTreeWidth;
	sWidthSumPair.sum= -(sttnA.otherVars.size()*sttnA.otherVars.size())
					   -(sttnB.otherVars.size()*sttnB.otherVars.size())
					   +(newAOtherVars.size()*newAOtherVars.size())
					   +(newBOtherVars.size()*newBOtherVars.size());
	return sWidthSumPair;
}

void CTabuSearch::MoveUp(int var){
	// note this method could be extremely improved by using an incremental version,
		//hint: heap and (heap2)-or-(degree-loc pair style) data-structure
	//entry condition: var-node is not root
	int locA,locB; locB=var2Loc[var];locA=sTabuTreeNodes[locB].parent;
	STabuTreeNode newA; STabuTreeNode& sttnA=sTabuTreeNodes[locA];
	STabuTreeNode newB; STabuTreeNode& sttnB=sTabuTreeNodes[locB];
	newA.parent=locB;
	newB.parent=sttnA.parent;
	newA.elimVar=sttnA.elimVar;
	newB.elimVar=sttnB.elimVar;

	//cout<<"sttnA.co+chi:"<<sttnA.childNodes.size()+sttnA.coveredCons.size()<<"  ";
	//cout<<"sttnA.co size "<<sttnA.coveredCons.size()<<"  ";

	bool inA;
	int elimVarA,elimVarB;
	vector<int>& newACoveredCons=newA.coveredCons;
	vector<int>& newAChildNodes=newA.childNodes;
	vector<int>& newAOtherVars=newA.otherVars;
	vector<int>& newBCoveredCons=newB.coveredCons;
	vector<int>& newBChildNodes=newB.childNodes;
	vector<int>& newBOtherVars=newB.otherVars;

	newACoveredCons=sttnA.coveredCons;
	newAChildNodes=sttnA.childNodes; // remember 'B' needs to be removed from this vector

	elimVarA=sttnA.elimVar; elimVarB=sttnB.elimVar;

	for(vector<int>::iterator itr=sttnB.coveredCons.begin();itr!=sttnB.coveredCons.end();itr++){
		vector<int>& hyperedge=cchg->GetHyperegde(*itr);
		inA=false;
		for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++)
			if(*itr2==elimVarA){
				inA=true; break;
			}

		if(inA)		newACoveredCons.push_back(*itr);
		else		newBCoveredCons.push_back(*itr);
	}
	
	for(vector<int>::iterator itr=sttnB.childNodes.begin();itr!=sttnB.childNodes.end();itr++){
		vector<int>& otherVars=sTabuTreeNodes[*itr].otherVars;
		inA=false;
		for(vector<int>::iterator itr2=otherVars.begin();itr2!=otherVars.end();itr2++)
			if(*itr2==elimVarA){
				inA=true; break;
			}

		if(inA)	newAChildNodes.push_back(*itr);
		else 	newBChildNodes.push_back(*itr);
	}
	
	//Note: there is a minor violation, since newBChildNodes has to contain newA-node

	//code to obtain newAOtherVars, 'remember that the old-B is not a child node of newA
	for(vector<int>::iterator itr=newACoveredCons.begin();itr!=newACoveredCons.end();itr++){
		vector<int>& hyperedge=cchg->GetHyperegde(*itr);
		for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++)
			if(*itr2!=elimVarA && !isPresent[*itr2]){
				isPresent[*itr2]=true; newAOtherVars.push_back(*itr2);
			}
	}
	for(vector<int>::iterator itr=newAChildNodes.begin();itr!=newAChildNodes.end();itr++){
		if(*itr==locB){  // skipping the old-B, its not a child node of newA
			*itr=newAChildNodes[newAChildNodes.size()-1];
			newAChildNodes.pop_back();
			itr--;	continue;
		}
		vector<int>& otherVars=sTabuTreeNodes[*itr].otherVars;
		for(vector<int>::iterator itr2=otherVars.begin();itr2!=otherVars.end();itr2++)
			if(!isPresent[*itr2] && *itr2!=elimVarA){
				isPresent[*itr2]=true; newAOtherVars.push_back(*itr2);
			}
	}

	for(int i=0,isize=newAOtherVars.size();i<isize;i++)
		isPresent[newAOtherVars[i]]=false;

	for(vector<int>::iterator itr=newBCoveredCons.begin();itr!=newBCoveredCons.end();itr++){
		vector<int>& hyperedge=cchg->GetHyperegde(*itr);
		for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++)
			if(*itr2!=elimVarB && !isPresent[*itr2]){
				isPresent[*itr2]=true; newBOtherVars.push_back(*itr2);
			}
	}

	for(vector<int>::iterator itr=newBChildNodes.begin();itr!=newBChildNodes.end();itr++){
		vector<int>& otherVars=sTabuTreeNodes[*itr].otherVars;
		for(vector<int>::iterator itr2=otherVars.begin();itr2!=otherVars.end();itr2++)
			if(!isPresent[*itr2] && *itr2!=elimVarB){
				isPresent[*itr2]=true; newBOtherVars.push_back(*itr2);
			}
	}
	
	for(vector<int>::iterator itr2=newAOtherVars.begin();itr2!=newAOtherVars.end();itr2++)
		if(!isPresent[*itr2] && *itr2!=elimVarB){
			isPresent[*itr2]=true; newBOtherVars.push_back(*itr2);
		}

	for(int i=0,isize=newBOtherVars.size();i<isize;i++)
		isPresent[newBOtherVars[i]]=false;
	
	newBChildNodes.push_back(locA); // since now, newA is a child node of newB
	
	sTabuTreeNodes[locA]=newA;
	sTabuTreeNodes[locB]=newB;
	
	for(vector<int>::iterator itr=newAChildNodes.begin();itr!=newAChildNodes.end();itr++)
		sTabuTreeNodes[*itr].parent=locA;
	for(vector<int>::iterator itr=newBChildNodes.begin();itr!=newBChildNodes.end();itr++)
		sTabuTreeNodes[*itr].parent=locB;

	if(newB.parent==-1)
		root=locB;
	else{
		for(vector<int>::iterator itr=sTabuTreeNodes[newB.parent].childNodes.begin();itr!=sTabuTreeNodes[newB.parent].childNodes.end();itr++)
			if(*itr==locA) 
				{*itr=locB; break;}
	}

	sTabuTreeNodes[root].parentalWidth=-1;
	FindSubTreeValues(root);
	sTabuTreeNodes[root].parentalQuadSum=0;
	UpdateParentalValues(root);
}


void CTabuSearch::Dummy(){
	SWidthSumPair sWidthSumPair;
	int rootVar;
	int minVar;
	SWidthSumPair minSWidthSumPair;

	cout<<endl;	
	do{
		RIPCheck();
		rootVar=sTabuTreeNodes[root].elimVar;  minVar=rootVar;
		minSWidthSumPair.width=sTabuTreeNodes[root].mySubTreeWidth;
		minSWidthSumPair.sum=0;
		cout<<"rootWidth: "<<sTabuTreeNodes[root].mySubTreeWidth<<" ";
		for(int i=0;i<numVars;i++){
			if(i==rootVar) continue;
			sWidthSumPair=Evaluate(i);
			if(   (minSWidthSumPair.width>sWidthSumPair.width)
				||(minSWidthSumPair.width==sWidthSumPair.width 
				&& minSWidthSumPair.sum>sWidthSumPair.sum    )){
				minVar=i;
				minSWidthSumPair=sWidthSumPair;
			}
		}
		if(minVar==rootVar) break;
		MoveUp(minVar);
		cout<<"Moveup: "<<minVar<<"   Width:  "<<minSWidthSumPair.width
			<<"   Sum:   "<<minSWidthSumPair.sum<<" newRootWidth:   "
			<<sTabuTreeNodes[root].mySubTreeWidth<<endl;
	}while(1);

	cout<<endl;
	cout<<"Last Width "<<sTabuTreeNodes[root].mySubTreeWidth<<endl;
}


void CTabuSearch::RIPCheck(){
	RIPNodeCheck(root);
	for(int i=0;i<numVars;i++)
		if(isEliminated[i]==false){
			cout<<"TabuRIPCheck, Error: var "<<i<<" not eliminated"<<endl;
			exit(1);
		}
		else
			isEliminated[i]=false;
	for(int i=0;i<numCons;i++)
		if(isCovered[i]==false){
			cout<<"TabuRIPCheck, Error: constraint "<<i<<" not covered"<<endl;
			exit(1);
		}
		else
			isCovered[i]=false;
}

void CTabuSearch::RIPNodeCheck(int node){
	STabuTreeNode& sttn=sTabuTreeNodes[node];
	for(vector<int>::iterator itr=sttn.childNodes.begin();itr!=sttn.childNodes.end();itr++)
		RIPNodeCheck(*itr);
	isPresent[sttn.elimVar]=true;
	for(vector<int>::iterator itr=sttn.otherVars.begin();itr!=sttn.otherVars.end();itr++)
		isPresent[*itr]=true;
	for(vector<int>::iterator itr=sttn.coveredCons.begin();itr!=sttn.coveredCons.end();itr++){
		isCovered[*itr]=true;
		vector<int>& hyperedge=cchg->GetHyperegde(*itr);
		for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++){
			if(!isPresent[*itr2]){
				cout<<"TabuRIPCheckNode, Error: constraint "<<*itr<<" not covered"<<endl;
				exit(1);
			}
		}
	}
	for(vector<int>::iterator itr=sttn.childNodes.begin();itr!=sttn.childNodes.end();itr++){
		vector<int>& otherVars=sTabuTreeNodes[*itr].otherVars;
		for(vector<int>::iterator itr2=otherVars.begin();itr2!=otherVars.end();itr2++){
			if(!isPresent[*itr2]){
				cout<<"TabuRIPCheckNode, Error: tree node "<<*itr<<" not covered by its parent"<<endl;
				cout<<"\t\t parent:"<<node<<endl;
				exit(1);
			}
		}
	}

	isEliminated[sttn.elimVar]=true; isPresent[sttn.elimVar]=false;
	for(vector<int>::iterator itr=sttn.otherVars.begin();itr!=sttn.otherVars.end();itr++)
		isPresent[*itr]=false;
}


void CTabuSearch::AddTabu(int var){
	int size=sTabuPairs[var].size();
	sTabuPairs[var].resize(size+1);
	sTabuPairs[var][size].var=sTabuTreeNodes[sTabuTreeNodes[var2Loc[var]].parent].elimVar;
	sTabuPairs[var][size].iteration=iteration;
}

bool CTabuSearch::IsSpecialTabu(int var){
	STabuTreeNode& sttnA=sTabuTreeNodes[sTabuTreeNodes[var2Loc[var]].parent];

	if((sttnA.childNodes.size()+sttnA.coveredCons.size()==1)
		&&(sttnA.coveredCons.size()==0))
		return true;
	else
		return false;	
}


bool CTabuSearch::IsTabu(int var){
	int parentVar=sTabuTreeNodes[sTabuTreeNodes[var2Loc[var]].parent].elimVar;
	vector<STabuPair>& tabus=sTabuPairs[parentVar];
	for(vector<STabuPair>::iterator itr=tabus.begin(); itr!=tabus.end();itr++){
		if(((*itr).iteration)<(iteration-tabuListSize)){
			*itr=tabus[tabus.size()-1];
			tabus.pop_back();
			itr--; continue;
		}
		else if(((*itr).var)==var)
			return true;
	}
	return false;
}


void CTabuSearch::Solve(float tabuListSizePIn, long maxIterationsIn, int cpuTimeLimitIn){
	// initialize all the required data members
	maxIterations=maxIterationsIn;
	tabuListSizeP=tabuListSizePIn;
	cpuTimeLimit=cpuTimeLimitIn;
	tabuListSize=(int)(numVars*tabuListSizeP);
	iteration=0;

	sTabuPairs.clear(); sTabuPairs.resize(numVars);
	//cout<<"CPU TIME  so far: "<<cpuTime()<<endl;
	double startTime=cpuTime();
	int minVar;
	SWidthSumPair minSWidthSumPair;
	long maxSumDiff=2*numVars*numVars;
	int rootVar;
	diversifyCalls=0;
	SWidthSumPair sWidthSumPair;
	while(iteration<maxIterations && (cpuTime()-startTime)<cpuTimeLimit){
		minSWidthSumPair.width=numVars;
		minSWidthSumPair.sum=maxSumDiff;
		rootVar=sTabuTreeNodes[root].elimVar;		
		for(int var=0;var<numVars;var++){
			if(var==rootVar) continue;
			if(IsSpecialTabu(var)) continue;
			if(IsTabu(var)) continue;
			sWidthSumPair=Evaluate(var);
			if(   sWidthSumPair.width<minSWidthSumPair.width 
				||(sWidthSumPair.width == minSWidthSumPair.width 
					&& sWidthSumPair.sum<minSWidthSumPair.sum)){
					minVar=var;
					minSWidthSumPair=sWidthSumPair;
			}
		}
		if(minSWidthSumPair.sum<0){
			AddTabu(minVar);
			MoveUp(minVar);
			//cout<<"Moveup: "<<minVar<<"   Width:  "<<minSWidthSumPair.width	<<"   Sum:   "<<minSWidthSumPair.sum<<" newRootWidth:   "	<<sTabuTreeNodes[root].mySubTreeWidth<<endl;
		}
		else{
			Diversify();
			diversifyCalls++;
		}
		
		if(sTabuTreeNodes[root].mySubTreeWidth<bestWidth){
			bestWidth=sTabuTreeNodes[root].mySubTreeWidth;
			//cout<<"bestWidth :"<<bestWidth<<"  itr:"<<iteration<<endl;
		}
		iteration++;
	}

    usedCpuTime=cpuTime()-startTime;
		
	//RIPCheck();
}

void CTabuSearch::PrintStats(){
	cout<<"Tabu Best Width : "<<bestWidth<<endl;
	cout<<"diversifyCalls : "<<diversifyCalls<<endl;
	cout<<"iteration : "<<iteration+1<<endl;
	cout<<"Tabu CPU time : "<<usedCpuTime<<endl;
}


void CTabuSearch::GetElimOrder(int loc, vector<int>& currElimOrder){
	STabuTreeNode sttn=sTabuTreeNodes[loc];
	for(vector<int>::iterator itr=sttn.childNodes.begin();itr!=sttn.childNodes.end();itr++)
		GetElimOrder(*itr,currElimOrder);
	currElimOrder.push_back(sttn.elimVar);
}


vector<int> CTabuSearch::GetElimOrder(){
	// the following code relies that there is only one root
	vector<int> currElimOrder;
	GetElimOrder(root,currElimOrder);
	if(currElimOrder.size()!=numVars){
		cout<<"CTabuSearch:GetElimOrder():Error: currElimOrder.size()!=numVars" <<endl;
		exit(1);
	}
	return currElimOrder;
}

void CTabuSearch::Diversify(){

	vector<int> currElimOrder=GetElimOrder();
	
	int maxVar=sTabuTreeNodes[0].elimVar;
	int maxSize=sTabuTreeNodes[0].otherVars.size();
	for(int loc=1;loc<numVars;loc++)
		if((sTabuTreeNodes[loc].otherVars.size())>maxSize){
			maxSize=sTabuTreeNodes[loc].otherVars.size(); 
			maxVar=sTabuTreeNodes[loc].elimVar;
		}
	//cout<<"Diversify initial maxSize "<<maxSize<<endl;

	int randomLoc=rand()%numVars;
	int maxLoc=0;

	for(int loc=0;loc<numVars;loc++)
		if(currElimOrder[loc]==maxVar)
			{maxLoc=loc; break;}
	if(randomLoc<maxLoc)
		for(int loc=maxLoc;loc>randomLoc;loc--)
			currElimOrder[loc]=currElimOrder[loc-1];
	else
		for(int loc=maxLoc;loc<randomLoc;loc++)
			currElimOrder[loc]=currElimOrder[loc+1];
	currElimOrder[randomLoc]=maxVar;
	ctd->SetElimOrder(currElimOrder);
	ctd->ElimOrder2TreeDe();

	// mega copy
	vector<STreeNode>& sTreeNodes=ctd->GetTreeNodes();
	root=-1;
	for(int i=0,isize=sTreeNodes.size();i<isize;i++){
		STreeNode& stn=sTreeNodes[i];
		STabuTreeNode& sttn=sTabuTreeNodes[i];
		sttn.childNodes=stn.childNodes;
		sttn.coveredCons=stn.coveredCons;
		sttn.elimVar=stn.elimVar;
		var2Loc[sttn.elimVar]=i;
		sttn.otherVars=stn.otherVars;
		sttn.parent=stn.parent;
		if(stn.parent==-1){
			if(root==-1){
				root=i;
			}
			else{
				cout<<"Error in Diversify \"CTabuSearch\": Unexpected case, more than one root in the tree decomposition."<<endl;
				cout<<"Current root value "<<root<<endl;
				exit(1);
			}
		}
	}
	sTabuTreeNodes[root].parentalWidth=-1;
	FindSubTreeValues(root);
	sTabuTreeNodes[root].parentalQuadSum=0;
	UpdateParentalValues(root);
	
	//sTabuPairs.clear(); sTabuPairs.resize(numVars);
}
