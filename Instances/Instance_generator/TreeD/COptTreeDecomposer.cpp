#include "COptTreeDecomposer.h"

COptTreeDecomposer::COptTreeDecomposer(CConstraintHypergraph* chg){
	cchg=chg;
}

void COptTreeDecomposer::Init(){
	startTime=cpuTime();

	COptTreeNode::cchg=cchg;
	COptTreeNode::numVars=cchg->GetNumVars();
	COptTreeNode::numCons=cchg->GetNumCons();
	COptTreeNode::ches=cchg->GetHyperedges();
	COptTreeNode::var2ches=cchg->GetVar2Hyperedges();

	COptTreeNode::callCount=0;
	COptTreeNode::recordGoods=true;
	COptTreeNode::isTouchedVar.resize(COptTreeNode::numVars);
	for(int loop=0;loop<COptTreeNode::numVars;loop++)
		COptTreeNode::isTouchedVar[loop]=0;
	COptTreeNode::isTouchedCon.resize(COptTreeNode::numCons);
	for(int loop=0;loop<COptTreeNode::numCons;loop++)
		COptTreeNode::isTouchedCon[loop]=0;
	COptTreeNode::tempInt=new int[COptTreeNode::numVars+COptTreeNode::numCons];
}


bool COptTreeDecomposer::Decompose(int k, double timeLimit){

	Init();
	COptTreeNode::timeLimit=timeLimit;
	COptTreeNode::startTime=startTime;
	COptTreeNode::k=k;

	vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents();
	cout<<"The number of connected components: "<<sCompVarsPairs.size()<<endl;

	COptTreeNode::timeLimitedStop=false;
	for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
		COptTreeNode cotn(*itr);
		if(!cotn.Decompose()){
			if(COptTreeNode::timeLimitedStop){
				cout<<endl<<" TIME OUT "<<endl;
				PrintStats();	return false;
			}
			else{
				cout<<endl<<"Not "<<k<<" Decomposable "<<endl;
				PrintStats();	return false;
			}
		}
	}

	cout<<endl<<k<<" Decomposable"<<endl;
	PrintStats();

	COptTreeDecomposition cotd(cchg,&COptTreeNode::cgd);
	cotd.PrintStats();
	if(cotd.RIPCheck())		cout<<" RIP Check success!"<<endl;

	return true;
}

bool COptTreeDecomposer::OptDecompose(double timeLimit, int startWidth){

	Init();
	COptTreeNode::timeLimit=timeLimit;
	COptTreeNode::startTime=startTime;

	vector<SCompVarsPair> sCompVarsPairs=cchg->GetConnectedComponents();
	cout<<"The number of connected components: "<<sCompVarsPairs.size()<<endl;

	if(sCompVarsPairs.size()!=1){
		cout<<" Special case , more than one connected component, not implemented yet"<<endl;
		cout<<" The code needs a full revamp to handle this .... "<<endl;
		exit(1);
	}

	COptTreeNode::timeLimitedStop=false;
	for(vector<SCompVarsPair>::iterator itr=sCompVarsPairs.begin();itr!=sCompVarsPairs.end();itr++){
		int k;
		if(startWidth==-1)	k=cchg->GetNumCons();
		else		     	k=startWidth;
		while(true){
			COptTreeNode::k=k;
			COptTreeNode cotn(*itr);
			if(cotn.Decompose()){
				COptTreeDecomposition cotd(cchg,&COptTreeNode::cgd);
				k=cotd.GetWidth();
				k--;
				if(k==0){
					cout<<" OptWidth = "<<1<<endl;
					return true;
				}
				// CLEAR ENTRIES OF WIDTH MORE THAN K FROM THE TWO DATABASES
				//COptTreeNode::cgd.DeleteRecords(k+1);
				//COptTreeNode::cngd.DeleteRecords(k+1);
				// THIS IS A QUICK HACK, CHECK THE EFFECT AND REVAMP THE CODE
				cout<<endl<<k+1<<"  Decomposable"<<endl; PrintStats();
				COptTreeNode::cgd.Clear();
				COptTreeNode::cngd.Clear();
			}
			else{
				if(COptTreeNode::timeLimitedStop==true){
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


void COptTreeDecomposer::PrintStats(){
	cout<<"Good Count :"<<COptTreeNode::cgd.RecordCount()<<endl;
	cout<<"NoGood Count :"<<COptTreeNode::cngd.RecordCount()<<endl;
	cout<<"OTN creationCount :"<<COptTreeNode::creationCount<<endl;
	usedCPUTime=cpuTime()-startTime;
	cout<<"usedCPUTime : "<<usedCPUTime<<endl;
	if(true){
		cout<<"num Good Records(check) : "<<COptTreeNode::cgd.CountRecords()<<endl;
		cout<<"num NoGood Records(check) : "<<COptTreeNode::cngd.CountRecords()<<endl;
	}
}
