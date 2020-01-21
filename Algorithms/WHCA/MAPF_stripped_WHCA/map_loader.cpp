//=======================================================================

#include "map_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <climits>
#include <float.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace boost;
using namespace std;

MapLoader::MapLoader(int rows, int cols) {
  map_filename = "NEW_EMPTY";
  int i,j;
  this->rows = rows;
  this->cols = cols;
  this->my_map = new bool[rows*cols];
  for (i=0; i<rows*cols; i++)
    this->my_map[i] = false;
  actions_offset = new int[5];
  actions_offset[0] = 0; actions_offset[1] = -cols; actions_offset[2] = 1; actions_offset[3] = cols; actions_offset[4] = -1; // [WAIT, NORTH, EAST, SOUTH, WEST]
  // add padding
  i=0;
  for (j=0; j<cols; j++)
    this->my_map[linearize_coordinate(i,j)] = true;
  i=rows-1;
  for (j=0; j<cols; j++)
    this->my_map[linearize_coordinate(i,j)] = true;
  j=0;
  for (i=0; i<rows; i++)
    this->my_map[linearize_coordinate(i,j)] = true;
  j=cols-1;
  for (i=0; i<rows; i++)
    this->my_map[linearize_coordinate(i,j)] = true;

}

MapLoader::MapLoader(string fname){
  map_filename = string(fname);
  string line;
  ifstream myfile (fname.c_str());
  bool* my_map;
  if (myfile.is_open()) {
    getline (myfile,line);
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tok(line, sep);
    tokenizer< char_separator<char> >::iterator beg=tok.begin();
    int rows = atoi ( (*beg).c_str() ); // read number of rows
    beg++;
    int cols = atoi ( (*beg).c_str() ); // read number of cols
    my_map = new bool[rows*cols];
    for (int i=0; i<rows*cols; i++)
      my_map[i] = false;

    // read map (and start/goal locations)
    for (int i=0; i<rows; i++) {
	getline (myfile, line);
	tokenizer< char_separator<char> > col_tok(line, sep);
	tokenizer< char_separator<char> >::iterator c_beg=col_tok.begin();
	int currCol = 0;
	for (int j=0; j<cols; j++) {
	  if ( (*c_beg).compare("S") == 0 ) {
	    my_map[cols*i + currCol] = false;
	    start_loc = cols*i + currCol;
	  } else if ( (*c_beg).compare("G") == 0 ) {
	    my_map[cols*i + currCol] = false;
	    goal_loc = cols*i + currCol;
	  } else {
	    int currCell = atoi ( (*c_beg).c_str() );
	    if (currCell == 1)
	      my_map[cols*i + currCol] = true;
	    else
	      my_map[cols*i + currCol] = false;
	  }
	  c_beg++;
	  currCol++;
	}
    }
    myfile.close();
    this->rows = rows;
    this->cols = cols;
    this->my_map = my_map;
    // initialize actions_offset array
    actions_offset = new int[5];
    actions_offset[0] = 0; actions_offset[1] = -cols; actions_offset[2] = 1; actions_offset[3] = cols; actions_offset[4] = -1;
  }
  else
    cerr << "Map file not found." << std::endl;
}

char* MapLoader::mapToChar() {
  char* mapChar = new char[rows*cols];
  for (int i=0; i<rows*cols; i++) {
    if ( i == start_loc )
      mapChar[i] = 'S';
    else if ( i == goal_loc )
      mapChar[i] = 'G';
    else if (this->my_map[i] == true)
      mapChar[i] = '*';
    else
      mapChar[i] = ' ';
  }
  return mapChar;
}

void MapLoader::printMap () {
  char* mapChar = mapToChar();
  printMap (mapChar);
  delete[] mapChar;
}


void MapLoader::printMap (char* mapChar) {
  cout << "MAP:";
  for (int i=0; i<rows*cols; i++) {
    if (i % cols == 0)
      cout << endl;
    cout << mapChar[i];
  }
  cout << endl;
}

void MapLoader::printHeuristic (const double* mapH, const int agent_id) {
  cout << endl << "AGENT "<<agent_id<<":";
  for (int i=0; i<rows*cols; i++) {
    if (i % cols == 0)
      cout << endl;
    if (mapH[i] == DBL_MAX)
      cout << "*,";
    else
      cout << mapH[i] << ",";
  }
  cout << endl;
}

bool* MapLoader::get_map() const {
  bool* retVal = new bool [ this->rows * this->cols ];
  memcpy (retVal, this->my_map, sizeof(bool)* this->rows * this->cols );
  return retVal;
}

MapLoader::~MapLoader() {
  delete[] this->my_map;
  delete[] this->actions_offset;
}

void MapLoader::saveToFile(std::string fname) {
  ofstream myfile;
  myfile.open (fname);
  myfile << rows << "," << cols << endl;
  for (int i=0; i<rows; i++) {
    for (int j=0; j<cols; j++) {
      if ( my_map[linearize_coordinate(i,j)] == true)
	myfile << "1";
      else
	myfile << "0";
      if (j<cols-1)
	myfile << ",";
      else
	myfile << endl;
    }
  }
  myfile.close();
}

void MapLoader::printPath(vector<int> path) {
  for (size_t i=0; i<path.size(); i++) {
    cout << "[" << row_coordinate(path[i]) << "," << col_coordinate(path[i]) << "] ; ";
  }
  cout << endl;
}

MapLoader::valid_actions_t MapLoader::get_action (int id1, int id2) const {
  int diff = id2-id1;
  valid_actions_t retVal = WAIT;
  if (diff == -cols)
    retVal = NORTH;
  if (diff == cols)
    retVal = SOUTH;
  if (diff == 1)
    retVal = EAST;
  if (diff == -1)
    retVal = WEST;
  return retVal;
}
