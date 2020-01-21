#ifndef TREED_H
#define TREED_H

#include <stdlib.h>
#include <time.h>
#include <string>
#include<iostream>
using namespace std;

#include "etreed.h"

/*--*----*---- STATIC MEMBERS IN CHYPERTREENODE ----*----*--*/
long CHypertreeNode::creationCount=0;
CConstraintHypergraph* CHypertreeNode::cchg;
int CHypertreeNode::numVars;
int CHypertreeNode::numCons;
int CHypertreeNode::k;
vector<vector<int> > CHypertreeNode::ches;
vector<vector<int> > CHypertreeNode::var2ches;
CDatabase CHypertreeNode::cngd;
CDatabase CHypertreeNode::cgd;
long CHypertreeNode::callCount;
bool CHypertreeNode::recordGoods;
bool CHypertreeNode::verbose;
bool CHypertreeNode::debug;
vector<int> CHypertreeNode::isTouchedVar;
vector<int> CHypertreeNode::touchedVars;
vector<int> CHypertreeNode::isTouchedCon;
vector<int> CHypertreeNode::touchedCons;

vector<vector<int> > CHypertreeNode::size2Cons;
int* CHypertreeNode::tempInt; 

bool CHypertreeNode::connectedHTD=true;
bool CHypertreeNode::isomorphicCompDetect=true;

bool CHypertreeNode::timeLimitedStop;
double CHypertreeNode::startTime;
double CHypertreeNode::timeLimit;
/*--*----*----*----*----*----*----*----*----*----*----*--*/

/*--*----*---- STATIC MEMBERS IN COPTTREENODE ----*----*--*/
long COptTreeNode::creationCount=0;
CConstraintHypergraph* COptTreeNode::cchg;
int COptTreeNode::numVars;
int COptTreeNode::numCons;
int COptTreeNode::k;
vector<vector<int> > COptTreeNode::ches;
vector<vector<int> > COptTreeNode::var2ches;
CDatabase COptTreeNode::cngd;
CDatabase COptTreeNode::cgd;
long COptTreeNode::callCount;
bool COptTreeNode::recordGoods;
bool COptTreeNode::verbose;
bool COptTreeNode::debug;
vector<int> COptTreeNode::isTouchedVar;
vector<int> COptTreeNode::touchedVars;
vector<int> COptTreeNode::isTouchedCon;
vector<int> COptTreeNode::touchedCons;

int* COptTreeNode::tempInt; 

bool COptTreeNode::connectedOTD=true;

bool COptTreeNode::timeLimitedStop;
double COptTreeNode::startTime;
double COptTreeNode::timeLimit;
/*--*----*----*----*----*----*----*----*----*----*----*--*/

#endif

