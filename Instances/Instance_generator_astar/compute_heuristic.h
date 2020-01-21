#ifndef COMPUTEHEURISTIC_H
#define COMPUTEHEURISTIC_H

#include <vector>
#include <utility>
#include <stdlib.h>
#include "egraph_reader.h"

using namespace std;

class ComputeHeuristic {
 public:
  double* h_vals;
  int goal_location;
  const bool* my_map;
  int map_size;
  const int* actions_offset;
  const EgraphReader* egr;
  double e_weight;
  
  ComputeHeuristic(int goal_location, const bool* my_map, int map_size, const int* actions_offset,
		   double e_weight, const EgraphReader* egr); // generate h_vals. (goal_location is linearized)
  double* getHVals(); // return a (deep) copy of h_vals;
  ~ComputeHeuristic();
};

#endif
