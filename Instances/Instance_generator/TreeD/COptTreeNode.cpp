#include "COptTreeNode.h"

COptTreeNode::COptTreeNode(SCompVarsPair sCVP){
    creationCount++;	sCompVarsPair=sCVP;	markVars.resize(numVars);
	for(int loop=0;loop<numVars;loop++) markVars[loop]=-2;	candVars.clear();
	for(vector<int>::iterator itr=sCompVarsPair.vars.begin();itr!=sCompVarsPair.vars.end();itr++){
		markVars[*itr]=-1;   candVars.push_back(*itr);
	}
	for(vector<int>::iterator itr=sCompVarsPair.sComp.cut.begin();itr!=sCompVarsPair.sComp.cut.end();itr++){
		markVars[*itr]=-1;   candVars.push_back(*itr);
	}
}

bool COptTreeNode::Decompose(){
	int candVar,con,var; newCut.clear();
	for(int loop=0;loop<candVars.size();loop++){
		candVar=candVars[loop];
		for(vector<int>::iterator itr=newCut.begin();itr!=newCut.end();itr++)  markVars[*itr]=-1;
		newCut.clear();
		for(vector<int>::iterator itr=var2ches[candVar].begin();itr!=var2ches[candVar].end();itr++){
			con=*itr;
			for(vector<int>::iterator itr2=ches[con].begin();itr2!=ches[con].end();itr2++){
				var=*itr2;
				if(markVars[var]==-1){
					newCut.push_back(var); markVars[var]=1;
				}
			}
		}

		int cutAdd=0;

		for(vector<int>::iterator itr2=sCompVarsPair.sComp.cut.begin();itr2!=sCompVarsPair.sComp.cut.end();itr2++){
			var=*itr2;
			if(markVars[var]==-1){
				newCut.push_back(var); markVars[var]=1; cutAdd++;
			}
		}
		if(newCut.size()>k+1) continue;
		if(sCompVarsPair.sComp.cut.size()>0)
			if(cutAdd==sCompVarsPair.sComp.cut.size() && connectedOTD) continue;
		vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents(sCompVarsPair.sComp, newCut);
		bool decomposable=true;
		if(debug)		cout<<" Has "<<sCompVarsPairs.size()<<"  children"<<endl;
		for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end() && decomposable;itr++){
			SCompVarsPair sCVP=*itr;
			if(cngd.IsPresent(sCVP.sComp))	decomposable=false;
			else if(cgd.IsPresent(sCVP.sComp)) continue;
			else{
				COptTreeNode chtn(sCVP);
				if(!chtn.Decompose())	decomposable=false;
			}
		}
		if(decomposable){
			cgd.AddGood(sCompVarsPair.sComp,newCut);  return true;
		}
		if(timeLimit!=0)
			if((cpuTime()-startTime)>timeLimit){
				timeLimitedStop=true; return false;
			}
	}
	cngd.AddNoGood(sCompVarsPair.sComp);  return false;
}

COptTreeNode::~COptTreeNode(){ }
