#ifndef _CDATABASE_H
#define _CDATABASE_H
#include <vector>
#include <map>
#include <utility>
#include <stdlib.h>
#include <iostream>
#include <assert.h>
using namespace std;

#include "SComp.h"
#define MAXKEYBLOCKSIZE 510

struct keyBlock{
	int size;//denotes the number of keys in the block
	int type; //0-internal 1-leaf
	int parent;
	int keys[MAXKEYBLOCKSIZE];
	int pointers[MAXKEYBLOCKSIZE+1];
};

struct dataBlock{
	int freeLoc;
	int data[4095];
};

class CDatabase{
	private:
		vector<keyBlock> keyBlocks;
		//vector<dataBlock> dataBlocks;
		map<int,vector<pair<SComp, vector<int> > > > mapCompsVec;
		int rootBlock;
		int recordCount;

		int numKeys; // these two entries are used by the checker
		int numRecords;

	public:
		CDatabase();


		void Add(SComp sComp,vector<int> cut);
		void AddNoGood(SComp& sComp);
		void AddGood(SComp& sComp,vector<int>& cut); // Note: cut here could be either a set of cons or vars

		bool IsPresent(SComp& sComp);
		//bool IsNoGood(SComp& sComp);
		//bool IsGood(SComp& sComp);

		vector<int> GetGoodCut(SComp sComp);

		int RecordCount() // returns the number of stored records
		{return recordCount;}

		int CountRecords(); // these two functions are used for checking
		void CountRecordsRec(int keyBlock);

		int k;
		int DeleteRecords(int k); // deletes all the entries of width at least k
		void DeleteRecordsRec(int keyBlock);
		void Clear(); //this is a quick hack, you need to write the above two functions
					  // remember! the above deletions might have to delete a record,
							//if any of its descendants violate the "k" condition



};
#endif
