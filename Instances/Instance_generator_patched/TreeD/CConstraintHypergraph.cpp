#include"CConstraintHypergraph.h"

void CConstraintHypergraph::ReadFile(char* filename){
	int scopeSize;
	int temp;
	ifstream in(filename);
	in>>numVars;
	in>>numCons;
	hyperedges.resize(numCons);
	var2Hyperedges.resize(numVars);
	for(int i=0;i<numCons;i++){
		in>>scopeSize;
		for(int j=0;j<scopeSize;j++){
			in>>temp;
			hyperedges[i].push_back(temp);
			var2Hyperedges[temp].push_back(i);
		}
	}
	in.close();
	InitComponentDetection(); // this is OK for now
}

void CConstraintHypergraph::SetHypergraph(int numVarsIn, int numConsIn, 
		  vector<vector<int> > hyperedgesIn){
    assert(numVars==-1 && numCons==-1);
	numVars=numVarsIn;
	numCons=numConsIn; 
	hyperedges=hyperedgesIn;
	int scopeSize;
	var2Hyperedges.resize(numVars);
	for(int i=0;i<numCons;i++){
		scopeSize=hyperedges[i].size();
		for(int j=0;j<scopeSize;j++)
			var2Hyperedges[hyperedges[i][j]].push_back(i);
	}
	InitComponentDetection(); // this is OK for now
}

void CConstraintHypergraph::Print(){
	cout<<"Printing HyperGraph..."<<endl;
	cout<<"numVars "<<numVars<<endl;
	cout<<"numCons "<<numCons<<endl;
	for(int i=0;i<numCons;i++){
		cout<<hyperedges[i].size()<<" : ";
		for(int j=0;j<hyperedges[i].size();j++)
			cout<<hyperedges[i][j]<<" ";
		cout<<endl;
	}
}

void CConstraintHypergraph::PrintMinMaxArity(){
	int maxArity=0; float meanArity=0.0;
	for(int i=0;i<numCons;i++){
		if(maxArity<hyperedges[i].size()) maxArity=hyperedges[i].size();
		meanArity+=hyperedges[i].size();
	}
	meanArity=meanArity/numCons;
	cout<<"MeanArity:"<<meanArity<<", MaxArity:"<<maxArity<<endl;
}


void CConstraintHypergraph::InitComponentDetection(){
	isTouchedVar.resize(numVars);
	for(int i=0;i<numVars;i++) isTouchedVar[i]=-2;
	isTouchedCon.resize(numCons);
	for(int i=0;i<numCons;i++)	isTouchedCon[i]=-2;
	isPushedVar.resize(numVars);
	for(int i=0;i<numVars;i++) isPushedVar[i]=0;

	con2Count.resize(numCons);
	for(int i=0;i<numCons;i++)	con2Count[i]=0;
	count2Cons.resize(numVars);
	isTouchedCount.resize(numVars);
	for(int i=0;i<numVars;i++) isTouchedCount[i]=0;
	tempCounts=new int[numVars];
}
pair<vector<SComp>, vector<vector<int> > >  CConstraintHypergraph::GetConnectedComponentsPairs(SComp sComp, vector<int> newCut){
	cout<<"CConstraintHypergraph::GetConnectedComponentsPairs is bugged "<<endl; exit(1);
	while(!varQ.empty()) varQ.pop();
	while(!conQ.empty()) conQ.pop();
	touchedVars.clear(); touchedCons.clear();

	for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
		isTouchedVar[*itr]=-3; // the vars in the cut are marked with -3
	varQ.push(sComp.firstElem);

	int var,con;

	while(varQ.size()!=0 || conQ.size()!=0){
		while(varQ.size()!=0){
			var=varQ.front();	varQ.pop();
			isTouchedVar[var]=-1; touchedVars.push_back(var);
			for(vector<int>::iterator itr=var2Hyperedges[var].begin();itr!=var2Hyperedges[var].end();itr++)
				if(isTouchedCon[*itr]==-2) conQ.push(*itr);
		}
		while(conQ.size()!=0){
			con=conQ.front();	conQ.pop();
			isTouchedCon[con]=-1; touchedCons.push_back(con);
			for(vector<int>::iterator itr=hyperedges[con].begin();itr!=hyperedges[con].end();itr++)
				if(isTouchedVar[*itr]==-2) varQ.push(*itr);
		}
	}

	for(vector<int>::iterator itr=newCut.begin();itr!=newCut.end();itr++)
		isTouchedVar[*itr]=-4;

	vector<SComp> sComps;   vector<vector<int> > compVars;
	int size;

	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++){
		if(isTouchedVar[*itr]==-1){
			size=sComps.size();
			SComp newSComp;
			newSComp.firstElem=*itr; varQ.push(*itr);
			compVars.resize(size+1);
			while(varQ.size()!=0 || conQ.size()!=0){
				while(varQ.size()!=0){
					var=varQ.front();	varQ.pop();
					assert(isTouchedVar[var]!=-2);
					isTouchedVar[var]=size; compVars[size].push_back(var);
					for(vector<int>::iterator itr=var2Hyperedges[var].begin();itr!=var2Hyperedges[var].end();itr++)
						if(isTouchedCon[*itr]==-1) conQ.push(*itr);
				}
				while(conQ.size()!=0){
					con=conQ.front();	conQ.pop();
					assert(isTouchedCon[con]!=-2);
					isTouchedCon[con]=size;
					for(vector<int>::iterator itr=hyperedges[con].begin();itr!=hyperedges[con].end();itr++)
						if(isTouchedVar[*itr]==-4 && isPushedVar[*itr]==0){
							newSComp.cut.push_back(*itr); isPushedVar[*itr]=1;
						}
						else if(isTouchedVar[*itr]==-1){
							if(*itr<newSComp.firstElem)	newSComp.firstElem=*itr;
							varQ.push(*itr);
						}
				}
			}
			sComps.push_back(newSComp);
			for(vector<int>::iterator itr=newSComp.cut.begin();itr!=newSComp.cut.end();itr++)
				isPushedVar[*itr]=0;
		}
	}

	for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=newCut.begin();itr!=newCut.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++)
		isTouchedCon[*itr]=-2;

	pair<vector<SComp>, vector<vector<int> > > sCompsVarsPair = make_pair(sComps,compVars);
	return sCompsVarsPair;
}


vector<SCompVarsPair>  CConstraintHypergraph::GetConnectedComponents(SComp sComp, vector<int> newCut){

	while(!varQ.empty()) varQ.pop();
	while(!conQ.empty()) conQ.pop();
	touchedVars.clear(); touchedCons.clear();

	for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
		isTouchedVar[*itr]=-3; // the vars in the cut are marked with -3
	varQ.push(sComp.firstElem);
	isTouchedVar[sComp.firstElem]=-1; touchedVars.push_back(sComp.firstElem);

	int var,con;

	while(varQ.size()!=0 || conQ.size()!=0){
		while(varQ.size()!=0){
			var=varQ.front();	varQ.pop();
			assert(isTouchedVar[var]==-1);
			for(vector<int>::iterator itr=var2Hyperedges[var].begin();itr!=var2Hyperedges[var].end();itr++)
				if(isTouchedCon[*itr]==-2) {
					conQ.push(*itr); isTouchedCon[*itr]=-1; touchedCons.push_back(*itr);
				}
		}
		while(conQ.size()!=0){
			con=conQ.front();	conQ.pop();
			assert(isTouchedCon[con]==-1);
			for(vector<int>::iterator itr=hyperedges[con].begin();itr!=hyperedges[con].end();itr++)
				if(isTouchedVar[*itr]==-2) {
					varQ.push(*itr); isTouchedVar[*itr]=-1; touchedVars.push_back(*itr);
				}
		}
	}

	for(vector<int>::iterator itr=newCut.begin();itr!=newCut.end();itr++)
		isTouchedVar[*itr]=-4;
	
	if(true){
		for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++){
			assert(isTouchedVar[*itr]==-4);		
		}
	}

	vector<SCompVarsPair> sCompVarsPairs;
	int size;

	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++){
		if(isTouchedVar[*itr]==-1){
			size=sCompVarsPairs.size();
			sCompVarsPairs.resize(size+1);
			SComp &newSComp=sCompVarsPairs[size].sComp;
			newSComp.firstElem=*itr; varQ.push(*itr);
			isTouchedVar[*itr]=size; sCompVarsPairs[size].vars.push_back(*itr);

			while(varQ.size()!=0 || conQ.size()!=0){
				while(varQ.size()!=0){
					var=varQ.front();	varQ.pop();
					assert(isTouchedVar[var]==size);
					for(vector<int>::iterator itr=var2Hyperedges[var].begin();itr!=var2Hyperedges[var].end();itr++)
						if(isTouchedCon[*itr]==-1) {
							conQ.push(*itr);isTouchedCon[*itr]=size;
						}
				}
				while(conQ.size()!=0){
					con=conQ.front();	conQ.pop();
					assert(isTouchedCon[con]==size);
					for(vector<int>::iterator itr=hyperedges[con].begin();itr!=hyperedges[con].end();itr++)
						if(isTouchedVar[*itr]==-4 && isPushedVar[*itr]==0){
							newSComp.cut.push_back(*itr);  isPushedVar[*itr]=1;
						}
						else if(isTouchedVar[*itr]==-1){
							if(*itr<newSComp.firstElem)	newSComp.firstElem=*itr;
							varQ.push(*itr);
							isTouchedVar[*itr]=size; sCompVarsPairs[size].vars.push_back(*itr);
						}
				}
			}
			for(vector<int>::iterator itr=newSComp.cut.begin();itr!=newSComp.cut.end();itr++)
				isPushedVar[*itr]=0;
		}
	}

	for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=newCut.begin();itr!=newCut.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++)
		isTouchedCon[*itr]=-2;

	if(!true){
		cerr<<endl;
		cerr<<"End of call to GetConnComponents :";
		cerr<<"Input comp :"; sComp.Print();
		cerr<<"size of newCut "<<newCut.size()<<endl;
		cerr<<"newCut : ";
		for(vector<int>::iterator itr=newCut.begin();itr!=newCut.end();itr++)
			cerr<<" "<<*itr; cerr<<endl;
		cerr<<"Results in num components : "<<sCompVarsPairs.size()<<endl;
		for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
			SCompVarsPair& sCVP=*itr;
			sCVP.sComp.Print();
		}
	}

	return sCompVarsPairs;
}


void CConstraintHypergraph::ExportGraph(char* filename){
	string ofn(filename);
	ofn+=".graph";
	ofstream out(ofn.c_str());

    vector<vector<int> > matrix;
	matrix.resize(numVars);
    for(int i=0;i<numVars;i++)   matrix[i].resize(numVars);
    for(int i=0;i<numCons;i++)
      {
        for(int j=0;j<hyperedges[i].size();j++)
          for(int k=j+1;k<hyperedges[i].size();k++)
            {
              matrix[hyperedges[i][j]][hyperedges[i][k]]=1;
              matrix[hyperedges[i][k]][hyperedges[i][j]]=1;
            }
      }

	int numEdges=0;
	for(int i=0;i<numVars;i++)
		for(int j=i+1;j<numVars;j++)
			if(matrix[i][j]==1) numEdges++;
	out<<numVars<<"   "<<numEdges<<endl;

	for(int i=0;i<numVars;i++)
		for(int j=i+1;j<numVars;j++)
			if(matrix[i][j]==1) out<<i+1<<"   "<<j+1<<endl;
	out.close();
}



void CConstraintHypergraph::ExportDBAI(char* filename){
	string ofn(filename);
	ofn+=".dbai";
	ofstream out(ofn.c_str());
	for(int i=0;i<numCons;i++){
		out<<"E"<<i<<"  (";
		for(int j=0;j<hyperedges[i].size();j++){
			out<<"V"<<hyperedges[i][j]<<" ";
			out<<((j+1<hyperedges[i].size())?" , ":"   ");
		}
		out<<((i<numCons-1)?"),":").")<<endl;
	}
	out.close();
}




vector<int> CConstraintHypergraph::GetConnectedConstraints(int variable){
	while(!varQ.empty()) varQ.pop();
	while(!conQ.empty()) conQ.pop();
	touchedVars.clear(); touchedCons.clear();
	varQ.push(variable); isTouchedVar[variable]=-1; touchedVars.push_back(variable);
	int var,con;
	while(varQ.size()!=0 || conQ.size()!=0){
		while(varQ.size()!=0){
			var=varQ.front();	varQ.pop();
			for(vector<int>::iterator itr=var2Hyperedges[var].begin();itr!=var2Hyperedges[var].end();itr++)
				if(isTouchedCon[*itr]==-2){
					conQ.push(*itr);isTouchedCon[*itr]=-1; touchedCons.push_back(*itr);
				}
		}
		while(conQ.size()!=0){
			con=conQ.front();	conQ.pop();
			for(vector<int>::iterator itr=hyperedges[con].begin();itr!=hyperedges[con].end();itr++)
				if(isTouchedVar[*itr]==-2){
					varQ.push(*itr);isTouchedVar[*itr]=-1; touchedVars.push_back(*itr);
				}
		}
	}

	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++)
		isTouchedCon[*itr]=-2;

	return touchedCons;
}

vector<int> CConstraintHypergraph::GetCoveringConstraints(vector<int>& vars, vector<int>& newVars, bool descendOrder){
  //TODO: check the effect of ordering the "result" based on different heuristics
	touchedVars.clear(); touchedCons.clear();
	for(vector<int>::iterator itr=vars.begin();itr!=vars.end();itr++)
		isTouchedVar[*itr]=-1;
	for(vector<int>::iterator itr=newVars.begin();itr!=newVars.end();itr++)
		isPushedVar[*itr]=1;
	int var,con; int numVarCovers; int numUniqueCovers=0;
	bool hasUniqueCovered; int redundantUniqueCovers=0;
	int numTwoCovers=0;	int numThreeCovers=0; int antiUniqueDeletions=0;
	bool flag;
	for(vector<int>::iterator itr=vars.begin();itr!=vars.end();itr++){
		var=*itr;
		hasUniqueCovered=false;
		for(vector<int>::iterator itr2=var2Hyperedges[var].begin();itr2!=var2Hyperedges[var].end();itr2++){
			con=*itr2;
			if(isTouchedCon[con]==-2){
				isTouchedCon[con]=-1;
				touchedCons.push_back(con);
				numVarCovers=0;
				for(vector<int>::iterator itr3=hyperedges[con].begin();itr3!=hyperedges[con].end();itr3++)
					if(isTouchedVar[*itr3]==-1) 	numVarCovers++;
				
				if(numVarCovers==1){ 
					numUniqueCovers++;
					if(!hasUniqueCovered){
						hasUniqueCovered=true;
						if(debug)	cout<<" "<<numVarCovers;					
					}
					else{
						flag=false;
						for(vector<int>::iterator itr3=hyperedges[con].begin();itr3!=hyperedges[con].end();itr3++)
							if(isPushedVar[*itr3]==1) 	{
								antiUniqueDeletions++;	flag=true; break;
							}
						if(flag==false){
							redundantUniqueCovers++; 
							isTouchedCon[con]=-2; 
							touchedCons.pop_back();
						}
					}
				}
				else if(debug) cout<<" "<<numVarCovers;
				if(numVarCovers==2) {
					numTwoCovers++;
					//cout<<"(";
					//for(vector<int>::iterator itr3=hyperedges[con].begin();itr3!=hyperedges[con].end();itr3++)
					//	if(isTouchedVar[*itr3]==-1) 	cout<<*itr3<<" ";
					//cout<<")";
				}
				if(numVarCovers==3) numThreeCovers++;
			}
		}
	}
	if(debug){
		cout<<endl;
		cout<<"numUniqueCovers:"<<numUniqueCovers<<"  redundantUniqueCovers:"<<redundantUniqueCovers
			<<"  numTwoCovers:"<<numTwoCovers
			<<"  numThreeCovers:"<<numThreeCovers<<endl;
		cout<<"antiUniqueDeletions:"<<antiUniqueDeletions<<endl;
	}

	assert((antiUniqueDeletions+redundantUniqueCovers)<=numUniqueCovers);
	
	if(false){
		cout<<"Printing the constraint-cover :"<<endl;
		for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++){
			con=*itr;
			cout<<"con:_"<<con<<" :  ";
				for(vector<int>::iterator itr3=hyperedges[con].begin();itr3!=hyperedges[con].end();itr3++){
					if(isTouchedVar[*itr3]==-1) 	cout<<*itr3<<" ";
					if(isPushedVar[*itr3]==1) 	cout<<"-"<<*itr3<<" ";
				}
			cout<<endl;
		}
	}
	
	if(descendOrder){
		// order the touchedCons in descending order of numCovers
		for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++){
			con=*itr;	numVarCovers=0;
			for(vector<int>::iterator itr3=hyperedges[con].begin();itr3!=hyperedges[con].end();itr3++)
				if(isTouchedVar[*itr3]==-1) 	numVarCovers++;
			if(isTouchedCount[numVarCovers]==0){
				isTouchedCount[numVarCovers]=1; touchedCounts.push_back(numVarCovers);
			}
			count2Cons[numVarCovers].push_back(con);
		}
		int size=touchedCounts.size();
		for(int loop=size-1;loop>=0;loop--)
			tempCounts[loop]=touchedCounts[loop];
		sort(tempCounts,size);
		int oldTouchedConsSize=touchedCons.size();
		touchedCons.clear();
		for(int loop=size-1;loop>=0;loop--){
			for(vector<int>::iterator itr=count2Cons[tempCounts[loop]].begin();
				itr!=count2Cons[tempCounts[loop]].end();itr++){
				touchedCons.push_back(*itr); 

				//cout<<" "<<tempCounts[loop]<<" ";

			}
			count2Cons[tempCounts[loop]].clear();
			isTouchedCount[tempCounts[loop]]=0;
		}
		touchedCounts.clear();
		if((oldTouchedConsSize!=touchedCons.size())){
			cout<<"oldTouchedConsSize "<<oldTouchedConsSize<<endl
				<<"touchedCons.size() "<<touchedCons.size()<<endl;
				assert((oldTouchedConsSize==touchedCons.size()));
		}
		//cout<<endl;
	}
	
	for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++)
		isTouchedCon[*itr]=-2;
	for(vector<int>::iterator itr=vars.begin();itr!=vars.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=newVars.begin();itr!=newVars.end();itr++)
		isPushedVar[*itr]=0;
	return touchedCons;
}

vector<int> CConstraintHypergraph::GetTouchingConstraints(vector<int>& vars, vector<int>& newVars){
	touchedVars.clear(); touchedCons.clear();
	int var,con; 
	for(vector<int>::iterator itr=vars.begin();itr!=vars.end();itr++){
		var=*itr;
		for(vector<int>::iterator itr2=var2Hyperedges[var].begin();itr2!=var2Hyperedges[var].end();itr2++){
			con=*itr2;
			if(isTouchedCon[con]==-2){
					isTouchedCon[con]=-1;
					touchedCons.push_back(con);
			}
		}
	}

	for(vector<int>::iterator itr=newVars.begin();itr!=newVars.end();itr++){
		var=*itr;
		for(vector<int>::iterator itr2=var2Hyperedges[var].begin();itr2!=var2Hyperedges[var].end();itr2++){
			con=*itr2;
			if(isTouchedCon[con]==-2){
					isTouchedCon[con]=-1;
					touchedCons.push_back(con);
			}
		}
	}

	for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++)
		isTouchedCon[*itr]=-2;
	return touchedCons;
}



vector<int> CConstraintHypergraph::GetFullyCoveredConstraints(vector<int>& vars){
	touchedVars.clear(); touchedCons.clear();
	for(vector<int>::iterator itr=vars.begin();itr!=vars.end();itr++)
		isTouchedVar[*itr]=-1;
	int var,con; 
	for(vector<int>::iterator itr=vars.begin();itr!=vars.end();itr++){
		var=*itr;
		for(vector<int>::iterator itr2=var2Hyperedges[var].begin();itr2!=var2Hyperedges[var].end();itr2++){
			con=*itr2;
			if(isTouchedCon[con]==-2){
				bool fullyCovered=true;
				for(vector<int>::iterator itr3=hyperedges[con].begin();itr3!=hyperedges[con].end();itr3++)
					if(isTouchedVar[*itr3]==-2){
						fullyCovered=false;break;
					}
				if(fullyCovered){
					isTouchedCon[con]=-1;
					touchedCons.push_back(con);
				}
			}
		}
	}

	for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++)
		isTouchedCon[*itr]=-2;
	for(vector<int>::iterator itr=vars.begin();itr!=vars.end();itr++)
		isTouchedVar[*itr]=-2;
	return touchedCons;
}

vector<int> CConstraintHypergraph::GetConnectedComponentsVars(){
	cout<<"CConstraintHypergraph::GetConnectedComponentsVars is bugged "<<endl; exit(1);
	touchedVars.clear();touchedCons.clear();
	while(!varQ.empty()) varQ.pop();
	while(!conQ.empty()) conQ.pop();
	int var,con;
	vector<int> firstElems;
	int size;
	for(int var=0;var<numVars;var++){
		if(isTouchedVar[var]==-2){
			size=firstElems.size();
			int firstElem=var; varQ.push(var);
			while(varQ.size()!=0 || conQ.size()!=0){
				while(varQ.size()!=0){
					var=varQ.front();	varQ.pop();
					isTouchedVar[var]=size; touchedVars.push_back(var);
					if(firstElem>var) firstElem=var;
					for(vector<int>::iterator itr=var2Hyperedges[var].begin();itr!=var2Hyperedges[var].end();itr++)
						if(isTouchedCon[*itr]==-2) conQ.push(*itr);
				}
				while(conQ.size()!=0){
					con=conQ.front();	conQ.pop();
					isTouchedCon[con]=size; touchedCons.push_back(con);
					for(vector<int>::iterator itr=hyperedges[con].begin();itr!=hyperedges[con].end();itr++)
						if(isTouchedVar[*itr]==-2) varQ.push(*itr);
				}
			}
			firstElems.push_back(firstElem);
		}
	}

	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++)
		isTouchedCon[*itr]=-2;
	return firstElems;
}


vector<SCompVarsPair> CConstraintHypergraph::GetConnectedComponents(){
	touchedVars.clear();touchedCons.clear();
	while(!varQ.empty()) varQ.pop();
	while(!conQ.empty()) conQ.pop();
	int var,con;
	vector<SCompVarsPair> sCompVarsPairs;
	int size;
	for(int loop=0;loop<numVars;loop++){
		if(isTouchedVar[loop]==-2){
			size=sCompVarsPairs.size();
			sCompVarsPairs.resize(size+1);
			int firstElem=loop; varQ.push(loop);
			isTouchedVar[loop]=size; touchedVars.push_back(loop);
			while(varQ.size()!=0 || conQ.size()!=0){
				while(varQ.size()!=0){
					var=varQ.front();	varQ.pop();
					if(firstElem>var) firstElem=var; sCompVarsPairs[size].vars.push_back(var);
					for(vector<int>::iterator itr=var2Hyperedges[var].begin();itr!=var2Hyperedges[var].end();itr++)
						if(isTouchedCon[*itr]==-2) {
							conQ.push(*itr);isTouchedCon[*itr]=size; touchedCons.push_back(*itr);
						}
				}
				while(conQ.size()!=0){
					con=conQ.front();	conQ.pop();
					for(vector<int>::iterator itr=hyperedges[con].begin();itr!=hyperedges[con].end();itr++)
						if(isTouchedVar[*itr]==-2) {
							varQ.push(*itr);isTouchedVar[*itr]=size; touchedVars.push_back(*itr);
						}
				}
			}
			sCompVarsPairs[size].sComp.firstElem=firstElem;
		}
	}

	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++)
		isTouchedVar[*itr]=-2;
	for(vector<int>::iterator itr=touchedCons.begin();itr!=touchedCons.end();itr++)
		isTouchedCon[*itr]=-2;
	return sCompVarsPairs;
}


vector<int> CConstraintHypergraph::GetVars(vector<int> lambda){
	touchedVars.clear();
	for(vector<int>::iterator itr=lambda.begin();itr!=lambda.end();itr++){
		int con=*itr;
		for(vector<int>::iterator itr2=hyperedges[con].begin();itr2!=hyperedges[con].end();itr2++){
			if(isTouchedVar[*itr2]==-2) {
				isTouchedVar[*itr2]=-1; touchedVars.push_back(*itr2);
			}
		}
	}
	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++)
		isTouchedVar[*itr]=-2;
	return touchedVars;
}

vector<int> CConstraintHypergraph::GetChi(vector<int> lambda, SCompVarsPair sCVP){
	touchedVars.clear(); 
	for(vector<int>::iterator itr=sCVP.sComp.cut.begin();itr!=sCVP.sComp.cut.end();itr++)
		isPushedVar[*itr]=1; 
	for(vector<int>::iterator itr=sCVP.vars.begin();itr!=sCVP.vars.end();itr++)
		isPushedVar[*itr]=1; 
	
	for(vector<int>::iterator itr=lambda.begin();itr!=lambda.end();itr++){
		int con=*itr;
		for(vector<int>::iterator itr2=hyperedges[con].begin();itr2!=hyperedges[con].end();itr2++){
			if(isPushedVar[*itr2]==1 && isTouchedVar[*itr2]==-2){
				isTouchedVar[*itr2]=-1; touchedVars.push_back(*itr2);
			}
		}
	}

	for(vector<int>::iterator itr=sCVP.sComp.cut.begin();itr!=sCVP.sComp.cut.end();itr++)
		isPushedVar[*itr]=0; 
	for(vector<int>::iterator itr=sCVP.vars.begin();itr!=sCVP.vars.end();itr++)
		isPushedVar[*itr]=0; 
	for(vector<int>::iterator itr=touchedVars.begin();itr!=touchedVars.end();itr++)
		isTouchedVar[*itr]=-2;

	return touchedVars;	
}



