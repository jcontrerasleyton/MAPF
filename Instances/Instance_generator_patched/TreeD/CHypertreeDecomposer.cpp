#include "CHypertreeDecomposer.h"

CHypertreeDecomposer::CHypertreeDecomposer(CConstraintHypergraph* chg){
	cchg=chg;
}

void CHypertreeDecomposer::Init(){
	startTime=cpuTime();

	CHypertreeNode::cchg=cchg;
	CHypertreeNode::numVars=cchg->GetNumVars();
	CHypertreeNode::numCons=cchg->GetNumCons();
	CHypertreeNode::ches=cchg->GetHyperedges();
	CHypertreeNode::var2ches=cchg->GetVar2Hyperedges();

	CHypertreeNode::callCount=0;
	CHypertreeNode::recordGoods=true;
	CHypertreeNode::isTouchedVar.resize(CHypertreeNode::numVars);
	for(int loop=0;loop<CHypertreeNode::numVars;loop++)
		CHypertreeNode::isTouchedVar[loop]=0;
	CHypertreeNode::isTouchedCon.resize(CHypertreeNode::numCons);
	for(int loop=0;loop<CHypertreeNode::numCons;loop++)
		CHypertreeNode::isTouchedCon[loop]=0;
	CHypertreeNode::tempInt=new int[CHypertreeNode::numVars+CHypertreeNode::numCons];
	CHypertreeNode::size2Cons.resize(CHypertreeNode::numVars);

}


bool CHypertreeDecomposer::Decompose(int k, double timeLimit){

	Init();
	CHypertreeNode::timeLimit=timeLimit;
	CHypertreeNode::startTime=startTime;
	CHypertreeNode::k=k;

	vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents();
	cout<<"The number of connected components: "<<sCompVarsPairs.size()<<endl;

	CHypertreeNode::timeLimitedStop=false;
	for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
		CHypertreeNode chtn(*itr);
		if(!chtn.Decompose()){
			if(CHypertreeNode::timeLimitedStop){
				cout<<endl<<" TIME OUT "<<endl;
				PrintStats();
				return false;
			}
			else{
				cout<<endl<<"Not "<<k<<" Decomposable "<<endl;
				PrintStats();
				return false;
			}
		}
	}

	cout<<endl<<k<<" Decomposable"<<endl;
	PrintStats();

	CHypertreeDecomposition chtd(cchg,&CHypertreeNode::cgd);
	chtd.PrintStats();
	if(chtd.RIPCheck())
		cout<<" RIP Check success!"<<endl;

	return true;
}

bool CHypertreeDecomposer::OptDecompose(double timeLimit, int startWidth){

	Init();
	CHypertreeNode::timeLimit=timeLimit;
	CHypertreeNode::startTime=startTime;

	vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents();
	cout<<"The number of connected components: "<<sCompVarsPairs.size()<<endl;

	if(sCompVarsPairs.size()!=1){
		cout<<" Special case , more than one connected component, not implemented yet"<<endl;
		cout<<" The code needs a full revamp to handle this .... "<<endl;
		exit(1);
	}

	CHypertreeNode::timeLimitedStop=false;
	for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
		int k;
		if(startWidth==-1)	k=cchg->GetNumCons();
		else		     	k=startWidth;
		while(true){
			CHypertreeNode::k=k;
			CHypertreeNode chtn(*itr);
			if(chtn.Decompose()){
				CHypertreeDecomposition chtd(cchg,&CHypertreeNode::cgd);
				k=chtd.GetWidth();
				k--;
				if(k==0){
					cout<<" OptWidth = "<<1<<endl;
					return true;
				}
				// CLEAR ENTRIES OF WIDTH MORE THAN K FROM THE TWO DATABASES
				//CHypertreeNode::cgd.DeleteRecords(k+1);
				//CHypertreeNode::cngd.DeleteRecords(k+1);
				// THIS IS A QUICK HACK, CHECK THE EFFECT AND REVAMP THE CODE
				cout<<endl<<k+1<<"  Decomposable"<<endl; PrintStats();
				chtd.PrintTreeWidth();
				CHypertreeNode::cgd.Clear();
				CHypertreeNode::cngd.Clear();
			}
			else{
				
				if(CHypertreeNode::timeLimitedStop==true){
					cout<<" TIME OUT: Width <= "<<k+1<<endl;
					PrintStats();
					return false;
				}else{
					cout<<endl<<" OptWidth = "<<k+1<<endl;
					PrintStats();
					return true;
				}
			}
		}
	}
}


void CHypertreeDecomposer::PrintStats(){
	cout<<"Good Count :"<<CHypertreeNode::cgd.RecordCount()<<endl;
	cout<<"NoGood Count :"<<CHypertreeNode::cngd.RecordCount()<<endl;
	cout<<"HTN creationCount :"<<CHypertreeNode::creationCount<<endl;
	usedCPUTime=cpuTime()-startTime;
	cout<<"usedCPUTime : "<<usedCPUTime<<endl;
	if(!true){
		cout<<"num Good Records(check) : "<<CHypertreeNode::cgd.CountRecords()<<endl;
		cout<<"num NoGood Records(check) : "<<CHypertreeNode::cngd.CountRecords()<<endl;
	}
}
