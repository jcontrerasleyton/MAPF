#include "CTreeDecomposition.h"

CTreeDecomposition::CTreeDecomposition(CConstraintHypergraph* chg){ 
	cchg=chg; 
	treeWidth=-1; 
	numVars=cchg->GetNumVars(); 
	numCons=cchg->GetNumCons();

	isPresent.resize(numVars);	for(int i=0;i<numVars;i++)	isPresent[i]=false;
	isEliminated.resize(numVars);	for(int i=0;i<numVars;i++)	isEliminated[i]=false;
	isCovered.resize(numCons);	for(int i=0;i<numCons;i++)	isCovered[i]=false;
}


void CTreeDecomposition::PrintDot(char* filename){
	// prints a DOT file
	//you may want to print elimination order and treewidth	.. 
	string dfn(filename);
	dfn+=".d0t";
	ofstream dotfile(dfn.c_str());
	dotfile<<"digraph G {"<<endl;
	for(int i=0,isize=sTreeNodes.size();i<isize;i++){
		STreeNode& stn=sTreeNodes[i];
		dotfile<<"_"<<i<<" [label="<<"v"<<stn.elimVar<<"s"<<stn.otherVars.size()<<"]"<<endl;
		dotfile<<"_"<<i<<" -> {";
			for(int j=0,jsize=stn.childNodes.size();j<jsize;j++){
				dotfile<<"_"<<stn.childNodes[j];
				if(j<jsize-1)
					dotfile<<"; ";
			}
			dotfile<<" }"<<endl;
	}
	dotfile<<"}"<<endl;
	dotfile.close();
}

void CTreeDecomposition::Print(){
	//prints in standard out
	cout<<" Print a tree decomposition ... ";
	STreeNode stn;
	cout<<"treeWidth: "<<treeWidth<<endl;
	for(int i=0,isize=sTreeNodes.size();i<isize;i++){
		stn=sTreeNodes[i];
		cout<<"TreeNode : "<<i<<endl;
		cout<<"  Parent : "<<stn.parent<<endl;
		cout<<"  Vars   : ";
		cout<<stn.elimVar<<" ";
		for(int j=0,jsize=stn.otherVars.size();j<jsize;j++)
			cout<<stn.otherVars[j]<<" ";
		cout<<endl;
		cout<<"  Covered Cons   : ";
		for(int j=0,jsize=stn.coveredCons.size();j<jsize;j++)
			cout<<stn.coveredCons[j]<<" ";
		cout<<endl;
		cout<<"  Child Nodes   : ";
		for(int j=0,jsize=stn.childNodes.size();j<jsize;j++)
			cout<<stn.childNodes[j]<<" ";
		cout<<endl;
	}
	cout<<endl;
	cout<<"Elimination Order: ";
	for(int i=0,isize=elimOrder.size();i<isize;i++)
		cout<<elimOrder[i]<<" ";
	cout<<endl;
	cout<<endl;
}

int CTreeDecomposition::AddTreeNode(STreeNode stn){
	//note: this code assumes that there are no freeLocs in sTreeNodes
	int currLoc=sTreeNodes.size();
	sTreeNodes.resize(currLoc+1);
	stn.parent=-1;
	sTreeNodes[currLoc]=stn;
	for(vector<int>::iterator itr=stn.childNodes.begin();itr!=stn.childNodes.end();itr++)
		sTreeNodes[*itr].parent=currLoc;
	return currLoc;
}

void CTreeDecomposition::ElimOrder2TreeDe(){
	//Elimination order to tree decomposition
	//elimination order needs to have been set properly, otherwise, in some cases, a default order will be chosen
	int numVars=cchg->GetNumVars();
	int numCons=cchg->GetNumCons();
	treeWidth=-1;
	sTreeNodes.clear();
	vector<vector<int> > buckets; // contains org- and new- hyperedges of a variable
	// an entry buckets[i][j] is an org-hyperedge iff the last bit of buckets[i][j] is zero
	buckets.resize(numVars);
	vector<int> var2Loc; //location of a variable in elimOrder
	vector<bool> isPresent; isPresent.resize(numVars);	for(int i=0;i<numVars;i++) isPresent[i]=false;
	STreeNode stn;
	int otherVarsSize; // this is a quick hack, donno why this is needed 

	//if no elimOrder, create default dummy one
	if(elimOrder.size()!=numVars){
		elimOrder.resize(numVars);	
		for(int i=0;i<numVars;i++)
			elimOrder[i]=i;
	}
	
	//place all the org- hyperedges in their buckets
	int minLoc;
	var2Loc.resize(numVars);
	for(int i=0;i<numVars;i++)
		var2Loc[elimOrder[i]]=i;
	for(int i=0;i<numCons;i++){
		minLoc=numVars;
		vector<int>& hyperedge=cchg->GetHyperegde(i);
		for(vector<int>::iterator itr=hyperedge.begin();itr!=hyperedge.end();itr++)
			if(minLoc>var2Loc[*itr]) minLoc=var2Loc[*itr];
		buckets[minLoc].push_back(i<<1);
		assert(buckets[minLoc][buckets[minLoc].size()-1]==(i+i));
	}
	
	//loop over elimOrder
	int currCon,otherVar;
	for(int i=0;i<numVars;i++){
		stn.elimVar=elimOrder[i];
		minLoc=numVars;
		stn.childNodes.clear();
		stn.otherVars.clear();
		stn.coveredCons.clear();
		//obtain stn.otherVars
		for(vector<int>::iterator itr=buckets[i].begin();itr!=buckets[i].end();itr++){
			currCon=*itr;
			if(currCon&0x1){
				//currCon is a new- hyperedge, i.e., a tree node loc
				currCon=currCon>>1;
				vector<int>& nodeVars=sTreeNodes[currCon].otherVars;
				stn.childNodes.push_back(currCon);
				for(vector<int>::iterator itr2=nodeVars.begin();itr2!=nodeVars.end();itr2++){
					otherVar=*itr2;
					if(!isPresent[otherVar] && otherVar!=stn.elimVar){
						isPresent[otherVar]=true;	stn.otherVars.push_back(otherVar);
						if(minLoc>var2Loc[otherVar]) minLoc=var2Loc[otherVar];
					}
				}
			}
			else{// (currCon & 0x1) == 0  , i.e., org- hyperedge
				currCon=currCon>>1;
				vector<int>& hyperedge = cchg->GetHyperegde(currCon);
				stn.coveredCons.push_back(currCon);
				for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++){
					otherVar=*itr2;
					if(!isPresent[otherVar] && otherVar!=stn.elimVar){
						isPresent[otherVar]=true;	stn.otherVars.push_back(otherVar);
						if(minLoc>var2Loc[otherVar]) minLoc=var2Loc[otherVar];
					}
				}
			}
		}
		
		// create a tree node
		otherVarsSize=stn.otherVars.size();
		if(otherVarsSize>treeWidth)
			treeWidth=otherVarsSize;

		int loc=AddTreeNode(stn);
		for(vector<int>::iterator itr=stn.otherVars.begin();itr!=stn.otherVars.end();itr++)
			isPresent[*itr]=false;
		loc=loc<<1;	loc|=0x1;
		if(minLoc!=numVars) buckets[minLoc].push_back(loc);
	}	
	UpdateRootNodes();
}


void CTreeDecomposition::SetElimOrder(vector<int>& varOrder){
	elimOrder=varOrder;
	ValidateElimOrder();
}

void CTreeDecomposition::ValidateElimOrder(){
	int numVars=cchg->GetNumVars();
	if(elimOrder.size()!=numVars){
		cout<<"Error in CTreeDecomposition::SetElimOrder:- varOrder.size()!=numVars "<<endl;
		exit(1);
	}
	vector<bool> isPresent;
	isPresent.resize(numVars);
	for(int i=0;i<numVars;i++)
		isPresent[i]=false;
	for(int i=0;i<numVars;i++)
		if(isPresent[elimOrder[i]]==true){
			cout<<"Error in CTreeDecomposition::SetElimOrder:- isPresent[elimOrder[i]]==true"<<endl;
			exit(1);
		}
		else
			isPresent[elimOrder[i]]=true;
}

void CTreeDecomposition::SimpleSifting(){
	// A simple implementation of the  sifting heuristic
	// this code requires a prior call to ElimOrder2TreeDe, preferably after MCS or MD
	// This code critically needs a revision, when you are in clear mood!!

 int numVars=cchg->GetNumVars();
  	
  cout<<"Num Vars is "<<numVars<<endl;
  vector<int> min_cvo;
  vector<int>& cvo=elimOrder;	
  min_cvo.resize(numVars);
  min_cvo=cvo;
  
  int min_elimwidth=treeWidth;
  bool changed;
   
 RESTART:
	
  changed=false;
 
  for(int i=0;i<numVars;i++){
    cout<<" "<<i;
    cout.flush();
    cvo=min_cvo;
    
    for(int j=0;j<numVars;j++){
        if(cvo[j]==i){
            for(int k=j;k>0;k--){
                cvo[k]=cvo[k-1];
            }
            cvo[0]=i; break;
        }
    }
    for(int pos=0;pos<numVars;pos++){

        if(pos!=0){
            cvo[pos-1]=cvo[pos];
            cvo[pos]=i;
        }
		
		ElimOrder2TreeDe();

        if(treeWidth<min_elimwidth){
			changed=true;
            min_elimwidth=treeWidth;
            min_cvo=cvo;
            cout<<" New Elim Width :-) "<< min_elimwidth<<endl;
        }
    }
    
  }
  
  if(changed) goto RESTART;
  
  cout<<"Elimination Width is "<<min_elimwidth<<endl;
   
}



void CTreeDecomposition::RemoveRedundantNodes(){
	bool success;
	int iteration=0;
	do{
		iteration++;
		success=false;
		vector<int> isTouchedVar; // 0-false 1-true
		isTouchedVar.resize(numVars);
		for(int loop=0;loop<numVars;loop++) isTouchedVar[loop]=0;
		for(int loop=0;loop<sTreeNodes.size();loop++){
			//cout<<"loop: "<<loop<<endl;
			int parent=sTreeNodes[loop].parent;		
			if(parent==-1) continue; // skip root nodes
			if(parent==-2) continue; // skip defunct nodes
			// note you skip the elimVar
			for(vector<int>::iterator itr=sTreeNodes[loop].otherVars.begin();itr!=sTreeNodes[loop].otherVars.end();  itr++)
				isTouchedVar[*itr]=1;
			int redundantParent=true;
			if(sTreeNodes[parent].elimVar>=numVars){
				cout<<"numVars: "<<numVars<<endl;
				cout<<"sTreeNodes[parent].elimVar: "<<sTreeNodes[parent].elimVar<<endl;
				exit(1);
			}
			assert(sTreeNodes[parent].elimVar>=0);
			if(isTouchedVar[sTreeNodes[parent].elimVar]==0) redundantParent=false;
			for(vector<int>::iterator itr=sTreeNodes[parent].otherVars.begin();itr!=sTreeNodes[parent].otherVars.end();  itr++)
				if(isTouchedVar[*itr]==0) redundantParent=false;
			if(redundantParent){
				success=true;
				for(vector<int>::iterator itr=sTreeNodes[parent].coveredCons.begin();itr!=sTreeNodes[parent].coveredCons.end();itr++){
					sTreeNodes[loop].coveredCons.push_back(*itr);
					//cout<<"CTreeDecomposition::RemoveRedundantNodes:: Assumption wrong!!"<<endl; exit(1);
				}
				for(vector<int>::iterator itr=sTreeNodes[parent].childNodes.begin();itr!=sTreeNodes[parent].childNodes.end();itr++)
					if(*itr!=loop){
						sTreeNodes[loop].childNodes.push_back(*itr);
						sTreeNodes[*itr].parent=loop;
					}
				sTreeNodes[loop].parent=sTreeNodes[parent].parent;
				if(sTreeNodes[parent].parent!=-1){
					int parentParent=sTreeNodes[parent].parent;
					for(vector<int>::iterator itr=sTreeNodes[parentParent].childNodes.begin();itr!=sTreeNodes[parentParent].childNodes.end();itr++)
						if(*itr==parent)    *itr=loop;
				}

				sTreeNodes[parent].parent=-2; // defunct node
				sTreeNodes[parent].elimVar=-1; sTreeNodes[parent].coveredCons.clear();
				sTreeNodes[parent].otherVars.clear(); sTreeNodes[parent].childNodes.clear();
			}
			for(vector<int>::iterator itr=sTreeNodes[loop].otherVars.begin();itr!=sTreeNodes[loop].otherVars.end();  itr++)
				isTouchedVar[*itr]=0;
		}

	}while(success);
	//cout<<"Redundant node deletions took "<<iteration<<" iterations "<<endl;
	DeleteDefunctNodes();
	UpdateRootNodes();
}

void CTreeDecomposition::DeleteDefunctNodes(){
	int unUsedLoc;
	for(int loop=0;loop<sTreeNodes.size();loop++)
		if(sTreeNodes[loop].parent==-2){
			unUsedLoc=loop; break;
		}
	for(int loop=unUsedLoc;loop<sTreeNodes.size();loop++){
		int parent=sTreeNodes[loop].parent;
		if(parent!=-2){
			sTreeNodes[unUsedLoc]=sTreeNodes[loop];
			if(parent!=-1)
				for(vector<int>::iterator itr=sTreeNodes[parent].childNodes.begin();itr!=sTreeNodes[parent].childNodes.end();itr++)
					if(*itr==loop) *itr=unUsedLoc; 
			for(vector<int>::iterator itr=sTreeNodes[loop].childNodes.begin();itr!=sTreeNodes[loop].childNodes.end();itr++)
				sTreeNodes[*itr].parent=unUsedLoc;
			sTreeNodes[loop].parent=-2; // make the node defunct
			for(int loop2=unUsedLoc;loop2<sTreeNodes.size();loop2++)
				if(sTreeNodes[loop2].parent==-2){
					unUsedLoc=loop2; break;
				}
		}
	}
	sTreeNodes.resize(unUsedLoc);
}


void CTreeDecomposition::UpdateRootNodes(){
	rootNodes.clear();
	for(int loop=0;loop<sTreeNodes.size();loop++)
		if(sTreeNodes[loop].parent==-1) rootNodes.push_back(loop);
}


void CTreeDecomposition::RIPCheck(){

	for(int loop=0; loop<sTreeNodes.size();loop++)
		if(sTreeNodes[loop].parent==-1)   RIPNodeCheck(loop);
	for(int i=0;i<numVars;i++)
		if(isEliminated[i]==false){
			cout<<"CTD-RIPCheck, Error: var "<<i<<" not eliminated"<<endl;
			cout<<"May be the var is not present in the any constraint "<<endl;		
			exit(1);
		}
		else
			isEliminated[i]=false;
	for(int i=0;i<numCons;i++)
		if(isCovered[i]==false){
			cout<<"CTD-RIPCheck, Error: constraint "<<i<<" not covered"<<endl;
			exit(1);
		}
		else
			isCovered[i]=false;

    
	  for(vector<int>::iterator itr=rootNodes.begin();itr!=rootNodes.end();itr++)
		isPresent[*itr]=true;

	  for(int loop=0;loop<sTreeNodes.size();loop++)
		  if(sTreeNodes[loop].parent==-1) assert(isPresent[loop]==true);
		  else							  assert(isPresent[loop]==false);

	  for(vector<int>::iterator itr=rootNodes.begin();itr!=rootNodes.end();itr++)
		isPresent[*itr]=false;

      //	  cout<<"RIP Check successful ... "<<endl; -- Removed by Liron ($$$)

}

void CTreeDecomposition::RIPNodeCheck(int node){

    //cerr<<"-1 ";
	
	/*
	if( node>=sTreeNodes.size()){
		cout<<"ERROR CTreeDecomposition::RIPNodeCheck::  node>=sTreeNodes.size() "<<endl;
		cout<<"node "<<node<<" sTreeNodes.size "<<sTreeNodes.size() <<endl;
	}*/

	STreeNode& sttn=sTreeNodes[node];
	
	//cerr<<" 0 ";
	//cerr<<"sttn.childNodes.size() "<<sttn.childNodes.size()<<endl;
	for(vector<int>::iterator itr=sttn.childNodes.begin();itr!=sttn.childNodes.end();itr++)
		RIPNodeCheck(*itr);
	

	//cerr<<" 0.1 ";

	assert(sttn.elimVar>=0 && sttn.elimVar<numVars);
	isPresent[sttn.elimVar]=true;

	//cerr<<" 0.2 ";
	
	for(vector<int>::iterator itr=sttn.otherVars.begin();itr!=sttn.otherVars.end();itr++){
		assert(*itr>=0 && *itr<numVars);
		isPresent[*itr]=true;
	}
	
	//cerr<<" 0.3 ";
	
	for(vector<int>::iterator itr=sttn.coveredCons.begin();itr!=sttn.coveredCons.end();itr++){
		assert(*itr>=0 && *itr<numCons);
		isCovered[*itr]=true;
		vector<int>& hyperedge=cchg->GetHyperegde(*itr);
		for(vector<int>::iterator itr2=hyperedge.begin();itr2!=hyperedge.end();itr2++){
			if(!isPresent[*itr2]){
				cout<<"CTD-RIPCheckNode, Error: constraint "<<*itr<<" not covered"<<endl;
				exit(1);
			}
		}
	}
	
	//cerr<<" 1 ";

	isPresent[sttn.elimVar]=false;
	for(vector<int>::iterator itr=sttn.otherVars.begin();itr!=sttn.otherVars.end();itr++)
		isPresent[*itr]=false;

	if(!isEliminated[sttn.elimVar])
		isEliminated[sttn.elimVar]=true; 
	else{
		cout<<"CHT-RIPNodeCheck::Error! variable sttn.elimVar already eliminated "<<endl;
		exit(1);
	}

	//cerr<<" 3 ";

	if(sttn.parent!=-1){
		isPresent[sTreeNodes[sttn.parent].elimVar]=true;
		for(vector<int>::iterator itr=sTreeNodes[sttn.parent].otherVars.begin();itr!=sTreeNodes[sttn.parent].otherVars.end();itr++)
			isPresent[*itr]=true;		
	}
	assert(isPresent[sttn.elimVar]==false);

	for(vector<int>::iterator itr=sttn.otherVars.begin();itr!=sttn.otherVars.end();itr++)
		if(isPresent[*itr]==false){
			//the var is forgotten at node
			if(!isEliminated[*itr])
				isEliminated[*itr]=true;
			else{
				cout<<"CHT-RIPNodeCheck::Error! variable sttn.otherVars[_] already eliminated "<<endl;
				exit(1);
			}
		}

	//cerr<<" 4' ";

	if(sttn.parent!=-1){
		isPresent[sTreeNodes[sttn.parent].elimVar]=false;
		for(vector<int>::iterator itr=sTreeNodes[sttn.parent].otherVars.begin();itr!=sTreeNodes[sttn.parent].otherVars.end();itr++)
			isPresent[*itr]=false;		
	}

	//cerr<<" 5 ";

}




