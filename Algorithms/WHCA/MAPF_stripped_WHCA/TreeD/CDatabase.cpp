#include "CDatabase.h"

CDatabase::CDatabase(){
	keyBlocks.resize(1);
	rootBlock=0;
	keyBlocks[rootBlock].size=0;
	keyBlocks[rootBlock].type=1;
	keyBlocks[rootBlock].parent=-1;
	recordCount=0;
	//cout<<"Size of keyBlock structure is "<<sizeof(keyBlock)<<endl;
	//cout<<"Size of dataBlock structure is "<<sizeof(dataBlock)<<endl;
}

void CDatabase::Clear(){
	keyBlocks.resize(1);
	rootBlock=0;
	keyBlocks[rootBlock].size=0;
	keyBlocks[rootBlock].type=1;
	keyBlocks[rootBlock].parent=-1;
	recordCount=0;
	mapCompsVec.clear(); // check the effect on memory usage 
	assert(CountRecords()==0);
}


void CDatabase:: Add(SComp sComp,vector<int> cut){

	recordCount++;
	int key;
	int sum=0;
	for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
		sum+=*itr;
	sComp.sum=sum;
	pair<SComp, vector<int> > sCompCutPair=make_pair(sComp,cut);
	key=sum+(sComp.firstElem*sComp.cut.size());
	int currKeyBlock=rootBlock;
	while(keyBlocks[currKeyBlock].type!=1){
		int matchLoc=0;
		while(key>=keyBlocks[currKeyBlock].keys[matchLoc] && matchLoc<keyBlocks[currKeyBlock].size)
			matchLoc++;
		keyBlocks[keyBlocks[currKeyBlock].pointers[matchLoc]].parent=currKeyBlock;
		currKeyBlock=keyBlocks[currKeyBlock].pointers[matchLoc];
	}

	int matchLoc=0;
	while(key>keyBlocks[currKeyBlock].keys[matchLoc] && matchLoc<keyBlocks[currKeyBlock].size)
		matchLoc++;

	if(matchLoc != keyBlocks[currKeyBlock].size && keyBlocks[currKeyBlock].keys[matchLoc]==key){
		mapCompsVec[key].push_back(sCompCutPair);
	}
	else if(MAXKEYBLOCKSIZE!=keyBlocks[currKeyBlock].size){
		// space available for insertion		
		for(int i=keyBlocks[currKeyBlock].size;i>matchLoc;i--)
			keyBlocks[currKeyBlock].keys[i]=keyBlocks[currKeyBlock].keys[i-1];
		keyBlocks[currKeyBlock].keys[matchLoc]=key;
		mapCompsVec[key].push_back(sCompCutPair);
		keyBlocks[currKeyBlock].size++;
		int parent=keyBlocks[currKeyBlock].parent;
		if(matchLoc==0 && parent!=-1){
			int oldFirstKey=keyBlocks[currKeyBlock].keys[1];
			for(int i=0;i<keyBlocks[parent].size;i++){
				if(keyBlocks[parent].keys[i]==oldFirstKey){
					keyBlocks[parent].keys[i]=key;
					cerr<<"CDatabase::Assumption wrong  1 "<<endl; exit(1);
					break;
				}
			}
		}
	}
	else{
		// create a new leaf block
		int newKeyBlock=keyBlocks.size();
		keyBlocks.resize(newKeyBlock+1);
		int half=(keyBlocks[currKeyBlock].size/2);
		keyBlocks[newKeyBlock].size=keyBlocks[currKeyBlock].size-half;
		for(int i=half;i<keyBlocks[currKeyBlock].size;i++)
			keyBlocks[newKeyBlock].keys[i-half]=keyBlocks[currKeyBlock].keys[i];
		keyBlocks[currKeyBlock].size=half;
		keyBlocks[newKeyBlock].type=1;

		if(matchLoc<half){
			for(int i=keyBlocks[currKeyBlock].size;i>matchLoc;i--)
				keyBlocks[currKeyBlock].keys[i]=keyBlocks[currKeyBlock].keys[i-1];
			keyBlocks[currKeyBlock].keys[matchLoc]=key;
			mapCompsVec[key].push_back(sCompCutPair);
			keyBlocks[currKeyBlock].size++;
			int parent=keyBlocks[currKeyBlock].parent;
			if(matchLoc==0 && parent!=-1){
				int oldFirstKey=keyBlocks[currKeyBlock].keys[1];
				for(int i=0;i<keyBlocks[parent].size;i++){
					if(keyBlocks[parent].keys[i]==oldFirstKey){
						keyBlocks[parent].keys[i]=key;
						cerr<<"CDatabase::Assumption wrong  2"<<endl; exit(1);
						break;
					}
				}
			}
		}
		else{
			matchLoc-=half;
			for(int i=keyBlocks[newKeyBlock].size;i>matchLoc;i--)
				keyBlocks[newKeyBlock].keys[i]=keyBlocks[newKeyBlock].keys[i-1];
			keyBlocks[newKeyBlock].keys[matchLoc]=key;
			mapCompsVec[key].push_back(sCompCutPair);
			keyBlocks[newKeyBlock].size++;
		}

		//change predecessors appropriately
		bool doParentUpdate=true;
		int parent=keyBlocks[currKeyBlock].parent;
		int childFirstKey=keyBlocks[newKeyBlock].keys[0];
		int childPointer=newKeyBlock;

		while(doParentUpdate){
			doParentUpdate=false;
			if(parent==-1){
				int newKeyBlock=keyBlocks.size();
				keyBlocks.resize(newKeyBlock+1);
				keyBlocks[newKeyBlock].parent=-1;
				keyBlocks[newKeyBlock].size=1;
				keyBlocks[newKeyBlock].type=0;
				keyBlocks[newKeyBlock].keys[0]=childFirstKey;
				keyBlocks[newKeyBlock].pointers[0]=rootBlock;
				keyBlocks[newKeyBlock].pointers[1]=childPointer;
				rootBlock=newKeyBlock;
				break;
			}

			matchLoc=0;
			while(childFirstKey>keyBlocks[parent].keys[matchLoc] && matchLoc<keyBlocks[parent].size)
				matchLoc++;

			if(keyBlocks[parent].size!=MAXKEYBLOCKSIZE){
				for(int i=keyBlocks[parent].size;i>matchLoc;i--){
					keyBlocks[parent].keys[i]=keyBlocks[parent].keys[i-1];
					keyBlocks[parent].pointers[i+1]=keyBlocks[parent].pointers[i];
				}
				keyBlocks[parent].keys[matchLoc]=childFirstKey;
				keyBlocks[parent].pointers[matchLoc+1]=childPointer;
				keyBlocks[parent].size++;
				int parentParent=keyBlocks[parent].parent;
				if(matchLoc==0 && parentParent!=-1){
					int oldFirstKey=keyBlocks[parent].keys[1];
					for(int i=0;i<keyBlocks[parentParent].size;i++){
						if(keyBlocks[parentParent].keys[i]==oldFirstKey){
							keyBlocks[parentParent].keys[i]=childFirstKey;
							cerr<<"CDatabase::Assumption wrong  3"<<endl; exit(1);
							break;
						}
					}
				}

			}
			else{

				//create a new internal block
				int newKeyBlock=keyBlocks.size();
				keyBlocks.resize(newKeyBlock+1);
				int half=(keyBlocks[parent].size/2);
				keyBlocks[newKeyBlock].size=keyBlocks[parent].size-half;
				
				for(int i=half;i<keyBlocks[parent].size;i++){
					keyBlocks[newKeyBlock].keys[i-half]=keyBlocks[parent].keys[i];
					keyBlocks[newKeyBlock].pointers[i-half+1]=keyBlocks[parent].pointers[i+1];
				}
				keyBlocks[parent].size=half;
				keyBlocks[newKeyBlock].type=0;

				if(matchLoc<half){
					for(int i=keyBlocks[parent].size;i>matchLoc;i--){
						keyBlocks[parent].keys[i]=keyBlocks[parent].keys[i-1];
						keyBlocks[parent].pointers[i+1]=keyBlocks[parent].pointers[i];
					}
					keyBlocks[parent].keys[matchLoc]=childFirstKey;
					keyBlocks[parent].pointers[matchLoc+1]=childPointer;
					keyBlocks[parent].size++;
					int parentParent=keyBlocks[parent].parent;
					if(matchLoc==0 && parentParent!=-1){
						int oldFirstKey=keyBlocks[currKeyBlock].keys[1];
						for(int i=0;i<keyBlocks[parentParent].size;i++){
							if(keyBlocks[parentParent].keys[i]==oldFirstKey){
								keyBlocks[parentParent].keys[i]=childFirstKey;
								cerr<<"CDatabase::Assumption wrong  4"<<endl; exit(1);
								break;
							}
						}
					}
				}
				else{
					matchLoc-=half;
					for(int i=keyBlocks[newKeyBlock].size;i>matchLoc;i--){
						keyBlocks[newKeyBlock].keys[i]=keyBlocks[newKeyBlock].keys[i-1];
						keyBlocks[newKeyBlock].pointers[i+1]=keyBlocks[newKeyBlock].pointers[i];
					}
					keyBlocks[newKeyBlock].keys[matchLoc]=childFirstKey;
					keyBlocks[newKeyBlock].pointers[matchLoc+1]=childPointer;
					keyBlocks[newKeyBlock].size++;
				}
				childFirstKey=keyBlocks[newKeyBlock].keys[0];	childPointer=newKeyBlock;
				for(int i=1;i<keyBlocks[newKeyBlock].size;i++){
					keyBlocks[newKeyBlock].keys[i-1]=keyBlocks[newKeyBlock].keys[i];
					keyBlocks[newKeyBlock].pointers[i-1]=keyBlocks[newKeyBlock].pointers[i];
				}
				keyBlocks[newKeyBlock].size--;
				keyBlocks[newKeyBlock].pointers[keyBlocks[newKeyBlock].size]=keyBlocks[newKeyBlock].pointers[keyBlocks[newKeyBlock].size+1];
				doParentUpdate=true;
				parent=keyBlocks[parent].parent;
			}
		}
	}

	if(!true){
		if(recordCount!=CountRecords()){
			cout<<"Erorr! recordCount!=CountRecords()  "<<endl;
			cout<<"recordCount: "<<recordCount<<"   countRecords: "<<CountRecords()<<endl;
			exit(1);
		}
		cout<<"numKeys: "<<numKeys<<endl;
	}
}

bool CDatabase::IsPresent(SComp& sComp){
	int key;
	int sum=0;
	for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
		sum+=*itr;
	sComp.sum=sum;
	key=sum+(sComp.firstElem*sComp.cut.size());
	int currKeyBlock=rootBlock;
	while(keyBlocks[currKeyBlock].type!=1){
		int matchLoc=0;
		while(key>=keyBlocks[currKeyBlock].keys[matchLoc] && matchLoc<keyBlocks[currKeyBlock].size)
			matchLoc++;
		keyBlocks[keyBlocks[currKeyBlock].pointers[matchLoc]].parent=currKeyBlock;
		currKeyBlock=keyBlocks[currKeyBlock].pointers[matchLoc];
	}

	int matchLoc=0;
	while(key>keyBlocks[currKeyBlock].keys[matchLoc] && matchLoc<keyBlocks[currKeyBlock].size)
		matchLoc++;

	if(matchLoc==keyBlocks[currKeyBlock].size)
		return false;

	vector<pair <SComp, vector<int> > > &cutVec=mapCompsVec[key];
	for(vector<pair <SComp, vector<int> > >::iterator itr=cutVec.begin();itr!=cutVec.end();itr++){
		pair <SComp, vector<int> > &currPair=*itr;
		SComp &currSComp=currPair.first;
		if((currSComp.cut.size()!=sComp.cut.size()) || (currSComp.firstElem!=sComp.firstElem) || (currSComp.sum!=sComp.sum) )
			continue;
		bool continueFurther=true;
		vector<int>::iterator itr2=sComp.cut.begin();
		for(vector<int>::iterator itr3=currSComp.cut.begin();itr3!=currSComp.cut.end() && continueFurther ;itr2++,itr3++)
			if((*itr2)!=(*itr3))
				continueFurther=false;
		if(continueFurther)
			return true;
	}
	return false;
}


void CDatabase::AddGood(SComp &sComp, vector<int> &cut){
	Add(sComp, cut);
	if(!true){
		int key; int sum=0;
		for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
			sum+=*itr;
		sComp.sum=sum;
		key=sum+(sComp.firstElem*sComp.cut.size());
		cerr<<"Adding Good key: "<<key<<endl;
		sComp.Print();
		cerr<<"recordCount: "<<recordCount<<endl;
		cerr<<"firstCon of con-cut "<<cut[0]<<endl;
		cerr<<"size of con-cut "<<cut.size()<<endl;
		cerr<<"con-cut : ";
		for(vector<int>::iterator itr=cut.begin();itr!=cut.end();itr++)
			cerr<<" "<<*itr; cerr<<endl<<endl;
	}

}

void CDatabase::AddNoGood(SComp &sComp){
	vector<int> cut;
	Add(sComp, cut);
	if(!true){
		int key; int sum=0;
		for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
			sum+=*itr;
		sComp.sum=sum;
		key=sum+(sComp.firstElem*sComp.cut.size());
		cerr<<"Adding NoGood key: "<<key<<endl;
		sComp.Print();
		cerr<<"recordCount: "<<recordCount<<endl;
	}
}

vector<int> CDatabase::GetGoodCut(SComp sComp){
	int key;
	int sum=0;
	for(vector<int>::iterator itr=sComp.cut.begin();itr!=sComp.cut.end();itr++)
		sum+=*itr;
	sComp.sum=sum;
	key=sum+(sComp.firstElem*sComp.cut.size());
	
	//cout<<"Searching for key :"<<key<<endl;

	int currKeyBlock=rootBlock;
	while(keyBlocks[currKeyBlock].type!=1){
		int matchLoc=0;
		while(key>=keyBlocks[currKeyBlock].keys[matchLoc] && matchLoc<keyBlocks[currKeyBlock].size)
			matchLoc++;
		keyBlocks[keyBlocks[currKeyBlock].pointers[matchLoc]].parent=currKeyBlock;
		currKeyBlock=keyBlocks[currKeyBlock].pointers[matchLoc];
	}

	int matchLoc=0;
	while(key>keyBlocks[currKeyBlock].keys[matchLoc] && matchLoc<keyBlocks[currKeyBlock].size)
		matchLoc++;

	if(matchLoc==keyBlocks[currKeyBlock].size){
		cout<<"Error: GOOD not found in GetGoodCut 1"<<endl; 
		cout<<"key: "<<key<<endl;
		sComp.Print();
		cout<<"recordCount: "<<recordCount<<endl;
		exit(1);
	}

	vector<pair <SComp, vector<int> > > &cutVec=mapCompsVec[key];
	for(vector<pair <SComp, vector<int> > >::iterator itr=cutVec.begin();itr!=cutVec.end();itr++){
		pair <SComp, vector<int> > &currPair=*itr;
		SComp &currSComp=currPair.first;
		if((currSComp.cut.size()!=sComp.cut.size()) || (currSComp.firstElem!=sComp.firstElem) || (currSComp.sum!=sComp.sum) )
			continue;
		bool continueFurther=true;
		vector<int>::iterator itr2=sComp.cut.begin();
		for(vector<int>::iterator itr3=currSComp.cut.begin();itr3!=currSComp.cut.end() && continueFurther ;itr2++,itr3++)
			if((*itr2)!=(*itr3))
				continueFurther=false;
		if(continueFurther){
			if(!true){
				cerr<<"found match: size of match : "<<currPair.second.size()<<endl;
				cerr<<"firstCon of cut "<<currPair.second[0]<<endl;
				cerr<<"size of con-cut "<<currPair.second.size()<<endl;
				cerr<<"con-cut : ";
				for(vector<int>::iterator itr=currPair.second.begin();itr!=currPair.second.end();itr++)
					cerr<<" "<<*itr; cerr<<endl;
			}
			return currPair.second;
		}
	}	
		cout<<"Error: GOOD not found in GetGoodCut 2"<<endl; 
		cout<<"key: "<<key<<endl;
		sComp.Print();
		cout<<"recordCount: "<<recordCount<<endl;
		exit(1);
}


int CDatabase::CountRecords(){
	numKeys=0; numRecords=0;
	CountRecordsRec(rootBlock);
	return numRecords;
}

void CDatabase::CountRecordsRec(int keyBlock){
	if(keyBlocks[keyBlock].type==0) // internal block
		for(int loop=0;loop<=keyBlocks[keyBlock].size;loop++)
			CountRecordsRec(keyBlocks[keyBlock].pointers[loop]);
	else{ // leaf block
		numKeys+=keyBlocks[keyBlock].size;
		for(int loop=0;loop<keyBlocks[keyBlock].size;loop++)
			numRecords+=(mapCompsVec[keyBlocks[keyBlock].keys[loop]]).size();
	}
}

int CDatabase::DeleteRecords(int width){
	k=width;
}

void CDatabase::DeleteRecordsRec(int keyBlock){

}
