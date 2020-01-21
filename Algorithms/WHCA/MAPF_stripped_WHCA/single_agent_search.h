#ifndef SINGLEAGENTSEARCH_H
#define SINGLEAGENTSEARCH_H

#include <stdlib.h>

#include <vector>
#include <list>
#include <utility>
#include "../../../include/sparsehash/dense_hash_map"


#include "node.h"
#include "egraph_reader.h"

using namespace std;
using google::dense_hash_map;


class SingleAgentSearch {
 public:
  // define typedefs (will also be used in ecbs_search)
  typedef boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::compare_node> > heap_open_t;
  typedef boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::secondary_compare_node> > heap_focal_t;
  typedef dense_hash_map<Node*, Node*, Node::NodeHasher, Node::eqnode> hashtable_t;
  // note -- hash_map (key is a node pointer, data is a node handler,
  //                   NodeHasher is the hash function to be used,
  //                   eqnode is used to break ties when hash values are equal)

  vector<int> path;  // a path that takes the agent from initial to goal location satisying all constraints
  // consider changing path from vector to deque (efficient front insertion)
  int path_cost;
  int start_location;
  int goal_location;
  const double* my_heuristic;  // this is the precomputed heuristic for this agent
  const bool* my_map;
  int map_size;
  const int* actions_offset;
  long long num_expanded;
  long long num_generated;
  //int op_updated;
  //int cl_updated;
  const EgraphReader* egr;
  bool tweak_g_val;
  double e_weight;
  int w_window_limit;
  int original;
  int cols;

  // note -- handle typedefs is defined inside the class (hence, include node.h is not enough).
  //  Node::open_handle_t open_handle;
  heap_open_t open_list;
  heap_open_t open_list_h;
  
  //  Node::focal_handle_t focal_handle;
  heap_focal_t focal_list;

  hashtable_t allNodes_table;
  hashtable_t allNodes_table_h;

  // used in hash table and would be deleted from the d'tor
  Node* empty_node;
  Node* deleted_node;

  SingleAgentSearch(int start_location, int goal_location, const bool* my_map, int map_size,
                    const int* actions_offset, const EgraphReader* egr = NULL, double e_weight = 1.0, bool tweak_g_val = false);
  // note if tweak_g_val is true, the costs are also inflated by e_weight

  const vector<int>* getPath() {return &path;}  // return a pointer to the path found;

  //  inline bool checkFutureConstraints(int goal_location, int curr_timestep, const vector< list< pair<int, int> > >* cons)

  int extractLastGoalTimestep(int goal_location, const vector< list< pair<int, int> > >* cons);

  inline void releaseClosedListNodes(hashtable_t* allNodes_table);

  inline void releaseClosedListNodes_h(hashtable_t* allNodes_table_h);

  inline bool isConstrained(int curr_id, int next_id, int next_timestep, const vector< list< pair<int, int> > >* cons);

  // returns true if a vaild path found (wrt my_map and constraints)
  // NOTE constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint)
  void updatePath(Node* goal);  // $$$ make inline?

  bool findPath(const vector < list< pair<int, int> > >* constraints, int w_window_limit, vector<pair<int,int>>& goals);

  int Manhattan(int a_location, int b_location);

  void InitializeRRA();
  bool ResumeRRA(int location);
  int AbstractDist(int location);
  bool isGoal(int curr, int next_id, int next_time, vector<pair<int,int>>& goals);
  bool futureConflict(int curr, int curr_time, const vector< list< pair<int, int> > >* cons);

  ~SingleAgentSearch();
};

#endif
