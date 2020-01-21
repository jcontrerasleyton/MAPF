#include "CHypertreeNode.h"

CHypertreeNode::CHypertreeNode(SCompVarsPair sCVP){
    creationCount++;
	if(debug){
		cout<<endl<<"sCVP.sComp.cut.size "<<sCVP.sComp.cut.size()<<endl;
		cout<<"sCVP.sComp.firstElem "<<sCVP.sComp.firstElem<<endl;
		cout<<"sCVP.vars.size "<<sCVP.vars.size()<<endl;
		cout<<" callCount "<<callCount<<endl;
	}

	sCompVarsPair=sCVP;
	markedRCVars=0;
	markedOtherVars=0;

	if(sCompVarsPair.sComp.cut.size()==0){
		connCons=cchg->GetConnectedConstraints(sCompVarsPair.sComp.firstElem);
		totalImpliedCons=k-1;
		rootHack=true;
	}
	else{
		rootHack=false;
		if(connectedHTD) 
			connCons=cchg->GetCoveringConstraints(sCompVarsPair.sComp.cut, sCompVarsPair.vars, false);
		else{
			connCons=cchg->GetTouchingConstraints(sCompVarsPair.sComp.cut, sCompVarsPair.vars);
			//for(vector<int>::iterator itr=connCons.begin();itr!=connCons.end();itr++)
			//	isTouchedCon[*itr]=1;
			//vector<int> oldConnCons=cchg->GetCoveringConstraints(sCompVarsPair.sComp.cut, sCompVarsPair.vars, false);
			//for(vector<int>::iterator itr=oldConnCons.begin();itr!=oldConnCons.end();itr++){
			//	if(isTouchedCon[*itr]!=1){
			//		cout<<" a con in oldConnCons is not in new connCons (normal HTD)"<<endl;
			//		cout<<"connCons. size "<<connCons.size()<<"   oldConnCons.size "<<oldConnCons.size()<<endl; 
			//		exit(1);
			//	}
			//}
			//for(vector<int>::iterator itr=connCons.begin();itr!=connCons.end();itr++)
			//	isTouchedCon[*itr]=0;
		}
		for(vector<int>::iterator itr=sCompVarsPair.sComp.cut.begin();itr!=sCompVarsPair.sComp.cut.end();itr++)
			isTouchedVar[*itr]=1;
		int loop=0; int var,con;

		if(debug)
			cout<<"connCons.size:"<<connCons.size()<<" \n ";
		
		for(vector<int>::iterator itr=connCons.begin();itr!=connCons.end();itr++,loop++){
			con=*itr;
			//cout<<" "<<con<<" ";
			for(vector<int>::iterator itr2=ches[con].begin();itr2!=ches[con].end();itr2++){
				var=*itr2;
				if(isTouchedVar[var]==1){
					var2cons[var].push_back(loop);
					//cout<<"var2cons["<<var<<"].push_back("<<loop<<")"<<endl;
				}
			}
		}
		for(vector<int>::iterator itr=sCompVarsPair.sComp.cut.begin();itr!=sCompVarsPair.sComp.cut.end();itr++)
			isTouchedVar[*itr]=0;
		totalImpliedCons=0;

		if((true || debug) && connectedHTD){
			CConstraintHypergraph checkhg;
			vector<vector<int> > tempHyperedges;
			for(vector<int>::iterator itr=sCompVarsPair.sComp.cut.begin();itr!=sCompVarsPair.sComp.cut.end();itr++){
				vector<int> &cons=var2cons[*itr];
				int size=tempHyperedges.size();
				tempHyperedges.resize(size+1);
				for(vector<int>::iterator itr2=cons.begin();itr2!=cons.end();itr2++){
					int con=*itr2;
					if(con<0){
						int newCon=(con*(-1))-1;
						tempHyperedges[size].push_back(newCon);
					}
					else
						tempHyperedges[size].push_back(con);
				}
			}

			checkhg.SetHypergraph(connCons.size(),sCompVarsPair.sComp.cut.size(),tempHyperedges);
			vector<SCompVarsPair> sCompVarsPairs=checkhg.GetConnectedComponents();
			//cout<<"connCons components: "<<sCompVarsPairs.size()<<endl;
			int size=0;
			int compSize;
			int compCount=1;
			for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
				SCompVarsPair sCVP2=*itr;
				size+=sCVP2.vars.size();
				//cout<<" "<<sCVP2.vars.size()<<" ";
				compSize=sCVP2.vars.size();
				if(isTouchedVar[compSize]==0){
					isTouchedVar[compSize]=1; touchedVars.push_back(compSize);
				}
				for(vector<int>::iterator itr2=sCVP2.vars.begin();itr2!=sCVP2.vars.end();itr2++){
					isTouchedCon[connCons[*itr2]]=compCount;
					size2Cons[compSize].push_back(connCons[*itr2]);
				}
				compCount++;
			}
			assert(connCons.size()==size);
			//cout<<endl;
			//for(vector<int>::iterator itr=connCons.begin();itr!=connCons.end();itr++)
			//	cout<<isTouchedCon[*itr]<<" ";
			//cout<<endl;

			size=touchedVars.size();
			for(int loop=0;loop<size;loop++){
				tempInt[loop]=touchedVars[loop];
				isTouchedVar[touchedVars[loop]]=0;
			}
			touchedVars.clear();
			sort(tempInt,size);
			int oldConnConsSize=connCons.size(); 
			//connCons.clear();
			vector<int> newConnCons;
			for(int loop=0;loop<size;loop++){
				compSize=tempInt[loop];
				for(vector<int>::iterator itr=size2Cons[compSize].begin();itr!=size2Cons[compSize].end();itr++)
					newConnCons.push_back(*itr);
				size2Cons[compSize].clear();
			}

			if(newConnCons.size()!=oldConnConsSize){
				cout<<"newConnCons.size "<<newConnCons.size()<<" oldConnConsSize "<<oldConnConsSize<<endl;
				exit(1);
			}

			//for(vector<int>::iterator itr=newConnCons.begin();itr!=newConnCons.end();itr++)
			//	cout<<isTouchedCon[*itr]<<" ";
			//cout<<endl;

			for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
				SCompVarsPair sCVP2=*itr;
				for(vector<int>::iterator itr2=sCVP2.vars.begin();itr2!=sCVP2.vars.end();itr2++)
					isTouchedCon[connCons[*itr2]]=0;
			}

			connCons=newConnCons;

			for(vector<int>::iterator itr=sCompVarsPair.sComp.cut.begin();itr!=sCompVarsPair.sComp.cut.end();itr++){
				isTouchedVar[*itr]=1;	var2cons[*itr].clear();
			}
		
			int loop=0;
			for(vector<int>::iterator itr=connCons.begin();itr!=connCons.end();itr++,loop++){
				con=*itr;
				//cout<<" "<<con<<" ";
				for(vector<int>::iterator itr2=ches[con].begin();itr2!=ches[con].end();itr2++){
					var=*itr2;
					if(isTouchedVar[var]==1){
						var2cons[var].push_back(loop);
						//cout<<"var2cons["<<var<<"].push_back("<<loop<<")"<<endl;
					}
				}
			}
			for(vector<int>::iterator itr=sCompVarsPair.sComp.cut.begin();itr!=sCompVarsPair.sComp.cut.end();itr++)
				isTouchedVar[*itr]=0;



		}
	}

	markVars.resize(numVars);
	for(int loop=0;loop<numVars;loop++) 	markVars[loop]=-2;
	for(vector<int>::iterator itr=sCompVarsPair.vars.begin();itr!=sCompVarsPair.vars.end();itr++)
		markVars[*itr]=-1;
	for(vector<int>::iterator itr=sCompVarsPair.sComp.cut.begin();itr!=sCompVarsPair.sComp.cut.end();itr++)
		markVars[*itr]=numCons<<1;

}

CHypertreeNode::~CHypertreeNode(){
//
}


bool CHypertreeNode::Decompose(){

	//cerr<<"Entered decomposition "<<endl;
	long beginCount=callCount;
	vector<int> implications; // contains any	con implied by current setup
	vector<vector<int> > excluded; // contains any con excluded at level "i" of the decision stack
	vector<int> decisions; // contains indices of connCons, which denote the selected constraints
	vector<vector<int> > implied; // contains any con implied at level "i" of the decision stack
	vector<int> connConsState; // tells whether the correspoding
								// con in connCons is 0-unMarked, 1-inDecision, 2-excluded, 3-implied, 4-subsumed
	vector<vector<int> > watches; // contains watches for a constraint, indexed by index of connCons

	vector<int> newCut; // to store the variables going to be in a new-cut

	connConsState.resize(connCons.size());
	excluded.resize(k+1);
	implied.resize(k+1);
	watches.resize(connCons.size());

	vector<int> &rc=sCompVarsPair.sComp.cut;

	for(vector<int>::iterator itr=rc.begin();itr!=rc.end();itr++){
		int v=*itr;
		//cerr<<"var: "<<v<<endl;
		vector<int> &cons=var2cons[v];
		if(cons.size()==1){
			implications.push_back(cons[0]);
			continue;
		}
		assert(cons.size()!=0);
		int c1=cons[0];
		int c2=cons[1];
		//cerr<<"cons size: "<<cons.size()<<"  c1:"<<c1<<"   c2:"<<c2<<endl;
		int newc1=(c1+1)*(-1);
		int newc2=(c2+1)*(-1);
		cons[0]=newc1;
		cons[1]=newc2;
		watches[c1].push_back(v);
		watches[c1].push_back(0);
		watches[c2].push_back(v);
		watches[c2].push_back(1);
	}

	for(int i=0,isize=connCons.size();i<isize;i++)
		connConsState[i]=0;
	bool anyZeroStateCon=true;

	while(1){
		anyZeroStateCon=true;
		while((totalImpliedCons+decisions.size()<k) && anyZeroStateCon && (markedRCVars!=rc.size() || markedOtherVars==0)){
			bool addedCon=false;
			while(!addedCon && implications.size()!=0){
				int pointer=implications[implications.size()-1];
				implications.pop_back();
				if(connConsState[pointer]!=0 && connConsState[pointer]!=3 && connConsState[pointer]!=4){
					cerr<<"Error, unexpected: connConsState[pointer]!=0, where pointer was implied  "<<endl;
					cerr<<" connConsState[pointer] : "<<connConsState[pointer]<<endl;
					exit(1);
				}
				int con=connCons[pointer];
				bool coversNew=false;
				for(vector<int>::iterator itr=ches[con].begin();itr!=ches[con].end();itr++){
					int v=*itr;
					if(markVars[v]<numCons && markedOtherVars==0 ){
						if(markVars[v]!=-2){
							if(markVars[v]==-1){
								coversNew=true;
							}
						}
					}else{
						if(markVars[v]==2*numCons){
							coversNew=true;
						}
					}
				}
				if(coversNew==false){
					connConsState[pointer]=4;
					continue;
				}

				connConsState[pointer]=3;
				implied[decisions.size()].push_back(pointer);
				totalImpliedCons++;

				addedCon=true;

				for(vector<int>::iterator itr=ches[con].begin();itr!=ches[con].end();itr++){
					int v=*itr;
					if(markVars[v]<numCons){
						if(markVars[v]!=-2){
							if(markVars[v]==-1){
								markedOtherVars++;
								markVars[v]=con; newCut.push_back(v);
							}
						}
					}else{
						if(markVars[v]==2*numCons){
							markedRCVars++;
							markVars[v]=numCons+con; newCut.push_back(v);
						}
					}
				}
			}

			if(addedCon)
				continue;

			int pointer;
			if(decisions.size()==0)
				pointer=0;
			else
				pointer=decisions[decisions.size()-1]+1;

			if(pointer==connCons.size()){
				anyZeroStateCon=false;
				continue;
			}

			while(!addedCon && anyZeroStateCon){
				if(connConsState[pointer]!=0){
					pointer++;
					if(pointer==connCons.size())
						anyZeroStateCon=false;
					continue;
				}
				int con=connCons[pointer];
				bool coversNew=false;
				for(vector<int>::iterator itr=ches[con].begin();itr!=ches[con].end();itr++){
					int v=*itr;
					if(markVars[v]<numCons && markedOtherVars==0){
						if(markVars[v]!=-2){
							if(markVars[v]==-1){
								coversNew=true;
							}
						}
					}else{
						if(markVars[v]==2*numCons){
							coversNew=true;
						}
					}
				}
				if(coversNew==false){
					connConsState[pointer]=4;
					pointer++;
					if(pointer==connCons.size())
						anyZeroStateCon=false;
					continue;
				}

				connConsState[pointer]=1;
				decisions.resize(decisions.size()+1);
				decisions[decisions.size()-1]=pointer;
				addedCon=true;
				pointer++;
				if(pointer==connCons.size())
					anyZeroStateCon=false;
				for(vector<int>::iterator itr=ches[con].begin();itr!=ches[con].end();itr++){
					int v=*itr;
					if(markVars[v]<numCons){
						if(markVars[v]!=-2){
							if(markVars[v]==-1){
								markedOtherVars++;
								markVars[v]=con; newCut.push_back(v);
							}
						}
					}else{
						if(markVars[v]==2*numCons){
							markedRCVars++;
							markVars[v]=numCons+con; newCut.push_back(v);
						}
					}
				}
			}
		}
		//cerr<<"Finished first part"<<endl;
		// Middle part
		callCount++;
		if( markedRCVars==rc.size() && markedOtherVars!=0){
			vector<SCompVarsPair>  sCompVarsPairs;
			sCompVarsPairs=cchg->GetConnectedComponents(sCompVarsPair.sComp, newCut);
			bool decomposable=true;
			if(debug)
				cout<<" Has "<<sCompVarsPairs.size()<<"  children"<<endl;
			
			if(!isomorphicCompDetect){
				// create currKString
				currKString.clear();
				for(int loop=0;loop<decisions.size();loop++){
					currKString.push_back(connCons[decisions[loop]]);
					for(vector<int>::iterator impItr=implied[loop].begin();impItr!=implied[loop].end();impItr++)
						currKString.push_back(connCons[*impItr]);
				}
				for(vector<int>::iterator impItr=implied[decisions.size()].begin();impItr!=implied[decisions.size()].end();impItr++)
					currKString.push_back(connCons[*impItr]);
			}

			for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end() && decomposable;itr++){
				SCompVarsPair sCVP=*itr;
				if(sCVP.vars.size()==0){
					cout<<"Error!  sCVP.vars.size()==0 ... catch this "<<endl;
					exit(1);
				}
				if(isomorphicCompDetect){
					if(cngd.IsPresent(sCVP.sComp))	decomposable=false;
					else if(cgd.IsPresent(sCVP.sComp)) continue;
					else{
						CHypertreeNode chtn(sCVP);
						if(!chtn.Decompose())	decomposable=false;
					}
				}else{
					// isomorphicCompDetect==false
					SComp newComp; 
					newComp.cut=currKString; newComp.firstElem=sCVP.sComp.firstElem;

					if(cngd.IsPresent(newComp))	decomposable=false;
					else if(cgd.IsPresent(newComp)) continue;
					else{
						CHypertreeNode chtn(sCVP);
						chtn.SetAlpha(currKString);
						if(!chtn.Decompose())	decomposable=false;
					}

				}
			}
			if(decomposable){
				currKString.clear();
				for(int loop=0;loop<decisions.size();loop++){
					currKString.push_back(connCons[decisions[loop]]);
					for(vector<int>::iterator impItr=implied[loop].begin();impItr!=implied[loop].end();impItr++)
						currKString.push_back(connCons[*impItr]);
				}
				for(vector<int>::iterator impItr=implied[decisions.size()].begin();impItr!=implied[decisions.size()].end();impItr++)
					currKString.push_back(connCons[*impItr]);
				if(!rootHack){
					if(!(currKString.size()==totalImpliedCons+decisions.size())){
						cerr<<"currKString size: "<<currKString.size()<<endl;
						cerr<<"totalImpliedCons: "<<totalImpliedCons <<endl;
						cerr<<"decisions.size(): "<<decisions.size() <<endl;
						cout<<"implied[decisions.size()].size: "<<implied[decisions.size()].size()<<endl;
						exit(1);
					}
				}
				if(isomorphicCompDetect)
					cgd.AddGood(sCompVarsPair.sComp,currKString);
				else{
					SComp newComp; 
					newComp.cut=alpha; newComp.firstElem=sCompVarsPair.sComp.firstElem;
					cgd.AddGood(newComp,currKString);
				}
				return true;
			}
		}

		//cout<<"markedRCVars "<<markedRCVars	<<"   rc.size()  "<<rc.size()<< "   markedOtherVars "<<  markedOtherVars<<endl;

		if(timeLimit!=0){
			if((cpuTime()-startTime)>timeLimit){
				timeLimitedStop=true; return false;
			}
		}

		//cerr<<"Finished middle part"<<endl;
		// End part
		if(false && callCount%10000==0){
				cerr<<"implications.size(): "<<implications.size()<<endl;
				cerr<<"decisions.size() is "<<decisions.size()<<endl;
				cerr<<"implied Cons  "<<totalImpliedCons<<endl;
				cerr<<"rc.size "<<rc.size()<<endl;
				cerr<<"markedRCVars "<<markedRCVars<<endl;
				cerr<<"markedOtherVars "<<markedOtherVars<<endl;
				cerr<<"callCount "<<callCount<<endl;
				cerr<<endl<<endl;
		}

		if( false &&  callCount-beginCount == 5000000){
			cerr<<endl<<connCons.size()<<" "<<rc.size()<<endl;
			for(vector<int>::iterator itr=rc.begin();itr!=rc.end();itr++){
				vector<int> &cons=var2cons[*itr];
				cerr<<cons.size()<<" ";
				for(vector<int>::iterator itr2=cons.begin();itr2!=cons.end();itr2++){
					int con=*itr2;
					if(con<0){
						int newCon=(con*(-1))-1;
						cerr<<newCon<<" ";
					}
					else
						cerr<<con<<" ";
				}
				cerr<<endl;
			}
			cout<<endl<<"Printed the small inside constraint hypergraph "<<endl;
			exit(1);
		}


		implications.clear();
		//remove decision

		if(decisions.size()==0)
			break;

		if(false && debug){
			//print sum of covered rc vars
			int sum=0;
			for(int i=0;i<numVars;i++){
				if(markVars[i]>numCons)
					if(markVars[i]!=2*numCons)
						sum+=i;
			}
			cerr<<sum<<" : ";
			sum=0;
			for(int i=0;i<connCons.size();i++){
				if(connConsState[i]==1 || connConsState[i]==3)
					sum+=i;
			}
			cerr<<sum<<" : ";

			cerr<<decisions.size()+totalImpliedCons<<" ";
			if(markedOtherVars!=0)
				cerr<<"1-other ";
			else
				cerr<<"0-other ";
			cerr<<" --  ";
			for(int i=0;i<numVars;i++){
				if(markVars[i]>numCons)
					if(markVars[i]!=2*numCons)
						cerr<<i<<" ";
			}
			cerr<<"  : ";
			cerr<<" covered cons are : ";
			for(int i=0;i<connCons.size();i++){
				if(connConsState[i]==1 || connConsState[i]==3)
					cerr<<i<<" ";
			}
			cerr<<endl;

		}

		int removedDecision=decisions[decisions.size()-1];
		decisions.pop_back();
		connConsState[removedDecision]=0;
		int con=connCons[removedDecision];
		for(vector<int>::iterator itr=ches[con].begin();itr!=ches[con].end();itr++){
			int v=*itr;
			if(markVars[v]<numCons){
				if(markVars[v]!=-2){
					if(markVars[v]==con){
						markedOtherVars--;
						markVars[v]=-1;  newCut.pop_back();
					}
				}
			}else{
				if(markVars[v]==(numCons+con)){
					markedRCVars--;
					markVars[v]=2*numCons; newCut.pop_back();
				}
			}
		}

		//remove exclusions
		for(int i=0,isize=excluded[decisions.size()+1].size();i<isize;i++)
			connConsState[excluded[decisions.size()+1][i]]=0;
		excluded[decisions.size()+1].clear();
		//remove implications
		for(int i=0,isize=implied[decisions.size()+1].size();i<isize;i++){
			int con=connCons[implied[decisions.size()+1][i]];
			connConsState[implied[decisions.size()+1][i]]=0;
			for(vector<int>::iterator itr=ches[con].begin();itr!=ches[con].end();itr++){
				int v=*itr;
				if(markVars[v]<numCons){
					if(markVars[v]!=-2){
						if(markVars[v]==con){
							markedOtherVars--;
							markVars[v]=-1; newCut.pop_back();
						}
					}
				}else{
					if(markVars[v]==(numCons+con)){
						markedRCVars--;
						markVars[v]=2*numCons; newCut.pop_back();
					}
				}
			}
		}

		totalImpliedCons-=implied[decisions.size()+1].size();
		implied[decisions.size()+1].clear();

		//unmark subsumptions below the highest decision
		if(false && decisions.size()==0){
			cerr<<"Error: decisions.size()==0 and you are trying to access decisions.size()-1 for resetting 4-state cons"<<endl;
			cerr<<"decisions[decisions.size()-1] "<<decisions[decisions.size()-1]<<endl;
			exit(1);
		}

		int resetStartPosition;
		if(decisions.size()!=0)
			resetStartPosition=decisions[decisions.size()-1]+1;
		else
			resetStartPosition=0;

		for(int i=resetStartPosition;i<connCons.size();i++)
			if(connConsState[i]==4)
				connConsState[i]=0;

		//add the previous decision as exclusion in the highest current excusionList and propagate
			//update implications during propagation
		excluded[decisions.size()].push_back(removedDecision);
		if(debug){
			if(decisions.size()==0)
				cout<<" "<<excluded[decisions.size()].size();
		}
		connConsState[removedDecision]=2;

		for(int i=0,isize=watches[removedDecision].size();i<isize;i+=2){
			int v=watches[removedDecision][i];
			int	l=watches[removedDecision][i+1];

			int otherWatchCon;
			int otherWatchLoc;
			bool loopFurther=true;
			vector<int> &cons=var2cons[v];
			for(int j=l+1,jsize=cons.size();j<jsize && loopFurther;j++){
				if(cons[j]<0){
					otherWatchLoc=j;
					otherWatchCon=(cons[j]*(-1))-1;
					if(connConsState[otherWatchCon]!=0){
						if(connConsState[otherWatchCon]==2){
							cerr<<"Alarm! your assumption wrong: connConsState[otherWatchCon]==2 "<<endl;
							cerr<<"otherWatchCon: "<<otherWatchCon<<endl;
							cerr<<"removedDecision "<<removedDecision<<endl;
							cerr<<"totalImpliedCons "<<totalImpliedCons<<endl;
							cerr<<"decisions.size() "<<decisions.size()<<endl;
							for(int k=0,ksize=decisions.size();k<=ksize;k++){
								cerr<<"Last excluded vector :"<<k <<": ";
								for(vector<int>::iterator itr=excluded[k].begin();itr!=excluded[k].end();itr++){
									cerr<<*itr<<" ";
								}
									cerr<<endl;
							}
							cerr<<"The var2cons is :";
							for(vector<int>::iterator itr=cons.begin();itr!=cons.end();itr++)
								cerr<<*itr<<" ";
							cerr<<endl;

							exit(1);
						}
						loopFurther=false;
					}
				}
				else if(connConsState[cons[j]]==2)   continue;
				else {
					assert((connConsState[cons[j]]==0)||(connConsState[cons[j]]==1)
						 ||(connConsState[cons[j]]==3)||(connConsState[cons[j]]==4));
					loopFurther=false;
					cons[l]=removedDecision;
					int otherCon=cons[j];
					cons[j]=(cons[j]+1)*(-1);

					watches[removedDecision][i+1]=watches[removedDecision][isize-1];
					watches[removedDecision][i]  =watches[removedDecision][isize-2];
					watches[removedDecision].pop_back();
					watches[removedDecision].pop_back();
					isize-=2;
					i-=2;

					watches[otherCon].push_back(v);
					watches[otherCon].push_back(j);
				}
			}

			for(int j=0;j<l && loopFurther;j++){
				if(cons[j]<0){
					otherWatchLoc=j;
					otherWatchCon=(cons[j]*(-1))-1;
					if(connConsState[otherWatchCon]!=0){
						if(connConsState[otherWatchCon]==2){
							cerr<<"Alarm! your assumption wrong: connConsState[cons[j]]==2 "<<endl;
							cerr<<"otherWatchCon: "<<otherWatchCon<<endl;
							cerr<<"removedDecision "<<removedDecision<<endl;
							cerr<<"totalImpliedCons "<<totalImpliedCons<<endl;
							cerr<<"decisions.size() "<<decisions.size()<<endl;
							for(int k=0,ksize=decisions.size();k<=ksize;k++){
								cerr<<"Last excluded vector :"<<k <<": ";
								for(vector<int>::iterator itr=excluded[k].begin();itr!=excluded[k].end();itr++){
									cerr<<*itr<<" ";
								}
									cerr<<endl;
							}
							cerr<<"The var2cons is :";
							for(vector<int>::iterator itr=cons.begin();itr!=cons.end();itr++)
								cerr<<*itr<<" ";
							cerr<<endl;

							exit(1);
						}
						loopFurther=false;
					}
				}
				else if(connConsState[cons[j]]==2)   continue;
				else {
					assert((connConsState[cons[j]]==0)||(connConsState[cons[j]]==1)
						 ||(connConsState[cons[j]]==3)||(connConsState[cons[j]]==4));
					loopFurther=false;
					cons[l]=removedDecision;
					int otherCon=cons[j];
					cons[j]=(cons[j]+1)*(-1);

					watches[removedDecision][i+1]=watches[removedDecision][isize-1];
					watches[removedDecision][i]  =watches[removedDecision][isize-2];
					watches[removedDecision].pop_back();
					watches[removedDecision].pop_back();
					isize-=2;
					i-=2;

					watches[otherCon].push_back(v);
					watches[otherCon].push_back(j);
				}
			}

			if(loopFurther)
				implications.push_back(otherWatchCon);

		}
		//cerr<<"Finished last part"<<endl;
	}
	//cerr<<"Exiting decompostion "<<endl;
	if(isomorphicCompDetect)
		cngd.AddNoGood(sCompVarsPair.sComp);
	else{
		SComp newComp;
		newComp.cut=alpha; newComp.firstElem=sCompVarsPair.sComp.firstElem;
		cngd.AddNoGood(newComp);
	}
	return false;
}
