#include "COptTreeDecomposition.h"

void COptTreeDecomposition::PrintDot(char* filename){
	// prints a DOT file
	//you may want to print elimination order and treewidth	..
	cout<<"NOT IMPLEMENTED YET "<<endl; exit(1);
	string dfn(filename);
	dfn+=".d0t";
	ofstream dotfile(dfn.c_str());
	dotfile<<"digraph G {"<<endl;
	//
	//
	dotfile<<"}"<<endl;
	dotfile.close();
}

void COptTreeDecomposition::PrintStats(){
	cout<<"Tree- width: "<<treeWidth<<endl;
}

void COptTreeDecomposition::Print(){
	//prints in standard out
	cout<<" Print a OptTree ... ";
	cout<<"________________________________________"<<endl;
	cout<<"\t"<<"root:";
	PrintStats();
	for(vector<int>::iterator itr=rootNodes.begin();itr!=rootNodes.end();itr++)
		cout<<" "<<*itr;
	cout<<endl;

	for(int loop=0,loopsize=nodes.size();loop<loopsize;loop++){
		cout<<loop<<" ";
		nodes[loop].Print();
	}
	cout<<endl;
	cout<<"________________________________________"<<endl;
	cout<<endl;
}


 COptTreeDecomposition::COptTreeDecomposition(CConstraintHypergraph *chgI, CDatabase *cgdI){
	cchg=chgI; cgd=cgdI;
	treeWidth=0;
	queue<int> unProcessed;
	vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents();
	int currNode, parent; vector<int> uLambda;
	for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
		SCompVarsPair& sCVP=*itr;
		currNode=nodes.size(); nodes.resize(currNode+1);
		nodes[currNode].chi=cgd->GetGoodCut(sCVP.sComp);
		if((nodes[currNode].chi.size()-1)>treeWidth) treeWidth=nodes[currNode].chi.size()-1;
		nodes[currNode].parent=-1;
		nodes[currNode].sComp=sCVP.sComp;
		unProcessed.push(currNode);
		rootNodes.push_back(currNode);
	}
	while(!unProcessed.empty()){
		parent=unProcessed.front(); unProcessed.pop();	//cout<<"Popped parent: "<<parent<<endl;
		uLambda=nodes[parent].chi;
		vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents(nodes[parent].sComp,uLambda);
		for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
			SCompVarsPair& sCVP=*itr;
			currNode=nodes.size(); nodes.resize(currNode+1);
			nodes[currNode].chi=cgd->GetGoodCut(sCVP.sComp);
			if((nodes[currNode].chi.size()-1)>treeWidth) treeWidth=nodes[currNode].chi.size()-1;
			nodes[currNode].parent=parent;
			nodes[currNode].sComp=sCVP.sComp;
			nodes[parent].children.push_back(currNode);
			unProcessed.push(currNode);
		}
	}
}


 bool COptTreeDecomposition::RIPCheck(){
	 int numVars=cchg->GetNumVars();
	 int numCons=cchg->GetNumCons();
	 isEliminated.resize(numVars);
	 for(int loop=0;loop<numVars;loop++)	 isEliminated[loop]=false;
	 isTouchedVar.resize(numVars);
	 for(int loop=0;loop<numVars;loop++)	 isTouchedVar[loop]=0;
	 isCovered.resize(numCons);
	 for(int loop=0;loop<numCons;loop++)	 isCovered[loop]=false;
	 for(vector<int>::iterator itr=rootNodes.begin();itr!=rootNodes.end();itr++)
		 RIPCheckNode(*itr);

	 for(int loop=0;loop<numCons;loop++){
		 if(isCovered[loop]==false){
			 if((cchg->GetHyperegde(loop)).size()!=0){
				 cout<<" RIPCheck error: The constraint "<<loop<<" is not covered "<<endl;
				 exit(1);
			 }
		 }
	 }
	 return true;
 }

 void COptTreeDecomposition::RIPCheckNode(int node){
	 for(vector<int>::iterator itr=nodes[node].children.begin();
		 itr!=nodes[node].children.end();itr++)
		RIPCheckNode(*itr);
	 int parent=nodes[node].parent;
	 if(parent!=-1)
		 for(vector<int>::iterator itr=nodes[parent].chi.begin();itr!=nodes[parent].chi.end();itr++)
			isTouchedVar[*itr]=1;
	 for(vector<int>::iterator itr=nodes[node].chi.begin();itr!=nodes[node].chi.end();itr++)
		 if(isTouchedVar[*itr]==0){
			 if(isEliminated[*itr]==true){
				//raise ALARM!
				 cout<<" RIPCheckNode error: The variable "<<*itr<<" is already eliminated "<<endl;
				 exit(1);
			 }else
				 isEliminated[*itr]=true;
		 }
	 if(parent!=-1)
		 for(vector<int>::iterator itr=nodes[parent].chi.begin();itr!=nodes[parent].chi.end();itr++)
			isTouchedVar[*itr]=0;
	 vector<int> coveredCons=cchg->GetFullyCoveredConstraints(nodes[node].chi);
	 for(vector<int>::iterator itr=coveredCons.begin();itr!=coveredCons.end();itr++)
		 isCovered[*itr]=true;
 }

