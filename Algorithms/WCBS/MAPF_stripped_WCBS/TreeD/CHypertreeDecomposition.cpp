#include "CHypertreeDecomposition.h"

void CHypertreeDecomposition::PrintDot(char* filename){
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

void CHypertreeDecomposition::PrintStats(){
	cout<<"Hyper- width: "<<width<<endl;
	cout<<"Tree- width: "<<treeWidth<<endl;
}

void CHypertreeDecomposition::PrintTreeWidth(){
	cout<<"Tree- width: "<<treeWidth<<endl;
}

void CHypertreeDecomposition::Print(){
	//prints in standard out
	cout<<" Print a hypertree ... ";
	cout<<"________________________________________"<<endl;
	cout<<"Width: "<<width<<endl;
	cout<<"\t"<<"root:";
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


 CHypertreeDecomposition::CHypertreeDecomposition(CConstraintHypergraph *chgI, CDatabase *cgdI){
	cchg=chgI; cgd=cgdI;
	width=0; treeWidth=0;
	queue<int> unProcessed;
	vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents();
	int currNode, parent; vector<int> uLambda;
	for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
		SCompVarsPair& sCVP=*itr;
		currNode=nodes.size(); nodes.resize(currNode+1);
		nodes[currNode].lambda=cgd->GetGoodCut(sCVP.sComp);
		if(!true){
			cerr<<"returned lambda size :"<<nodes[currNode].lambda.size()<<endl;
			cerr<<"lambda : ";
			for(vector<int>::iterator itr=nodes[currNode].lambda.begin();itr!=nodes[currNode].lambda.end();itr++)
				cerr<<" "<<*itr; cerr<<endl;
		}
		if(nodes[currNode].lambda.size()>width) width=nodes[currNode].lambda.size();
		nodes[currNode].chi=cchg->GetChi(nodes[currNode].lambda,sCVP);
		if((nodes[currNode].chi.size()-1)>treeWidth) treeWidth=nodes[currNode].chi.size()-1;
		nodes[currNode].parent=-1;
		nodes[currNode].sComp=sCVP.sComp;
		unProcessed.push(currNode);
		rootNodes.push_back(currNode);
	}
	while(!unProcessed.empty()){
		parent=unProcessed.front(); unProcessed.pop();
		//cout<<"Popped parent: "<<parent<<endl;
		uLambda=cchg->GetVars(nodes[parent].lambda);
		vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents(nodes[parent].sComp,uLambda);
		for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
			SCompVarsPair& sCVP=*itr;
			currNode=nodes.size(); nodes.resize(currNode+1);
			if(CHypertreeNode::isomorphicCompDetect)
				nodes[currNode].lambda=cgd->GetGoodCut(sCVP.sComp);
			else{
				SComp newComp;
				newComp.cut=nodes[parent].lambda; newComp.firstElem=sCVP.sComp.firstElem;
				nodes[currNode].lambda=cgd->GetGoodCut(newComp);
			}
			if(!true){
				cerr<<"returned lambda size :"<<nodes[currNode].lambda.size()<<endl;
				cerr<<"lambda : ";
				for(vector<int>::iterator itr=nodes[currNode].lambda.begin();itr!=nodes[currNode].lambda.end();itr++)
					cerr<<" "<<*itr; cerr<<endl;
			}
			if(nodes[currNode].lambda.size()>width) width=nodes[currNode].lambda.size();
			nodes[currNode].chi=cchg->GetChi(nodes[currNode].lambda,sCVP);
			if((nodes[currNode].chi.size()-1)>treeWidth) treeWidth=nodes[currNode].chi.size()-1;
			nodes[currNode].parent=parent;
			nodes[currNode].sComp=sCVP.sComp;
			nodes[parent].children.push_back(currNode);
			unProcessed.push(currNode);
		}
	}
}


 bool CHypertreeDecomposition::RIPCheck(){
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

 void CHypertreeDecomposition::RIPCheckNode(int node){
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

