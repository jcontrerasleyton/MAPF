#ifndef SINGLEAGENTECBS_H
#define SINGLEAGENTECBS_H

#include <stdlib.h>

#include <vector>
#include <list>
#include <utility>
#include <random>

#include "node.h"
#include "egraph_reader.h"

using std::cout;

class SingleAgentECBS {
 public:
  // define typedefs (will also be used in ecbs_search)
  typedef boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::compare_node> > heap_open_t;
  typedef boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::secondary_compare_node> > heap_focal_t;
  //typedef boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::secondary_hwy_compare_node> > heap_focal_t;

  typedef dense_hash_map<Node*, Node*, Node::NodeHasher, Node::eqnode> hashtable_t;
  // note -- hash_map (key is a node pointer, data is a node handler,
  //                   NodeHasher is the hash function to be used,
  //                   eqnode is used to break ties when hash values are equal)
  
  vector<int> path;  // a path that takes the agent from initial to goal location satisying all constraints
  // consider changing path from vector to deque (efficient front insertion)
  double path_cost;
  int start_location;
  int goal_location;
  const double* my_heuristic;  // this is the precomputed heuristic for this agent
  const bool* my_map;
  int map_size;
  const int* actions_offset;
  uint64_t num_expanded;
  uint64_t num_generated;
  const EgraphReader* egr;
  bool tweak_g_val;
  double e_weight;  // EGRAPH's inflation factor
  double lower_bound;  // FOCAL's lower bound ( = e_weight * min_f_val)
  double min_f_val;  // min f-val seen so far
  int rrr_it;
  const bool rand_succ_gen;

  vector<int> directions = {0, 1, 2, 3, 4};
  std::mt19937 rand_num_gen;


  // note -- handle typedefs is defined inside the class (hence, include node.h is not enough).
  //  Node::open_handle_t open_handle;
  heap_open_t open_list;

  //  Node::focal_handle_t focal_handle;
  heap_focal_t focal_list;

  hashtable_t allNodes_table;

  // used in hash table and would be deleted from the d'tor
  Node* empty_node;
  Node* deleted_node;

  /* ctor
   */
  SingleAgentECBS(int start_location, int goal_location,
                  const double* my_heuristic,
                  const bool* my_map, int map_size, const int* actions_offset,
                  const EgraphReader* egr = NULL, double e_weight = 1.0, bool tweak_g_val = false, int rrr_it = 0, const bool rand_succ_gen = false);
  // note if tweak_g_val is true, the costs are also inflated by e_weight

  /* return a pointer to the path found.
   */
  const vector<int>* getPath() {return &path;}  // return a pointer to the path found;

  /* returns the minimal plan length for the agent (that is, extract the latest timestep which
     has a constraint invloving this agent's goal location).
  */
  int extractLastGoalTimestep(int goal_location, const vector< list< pair<int, int> > >* cons);

  inline void releaseClosedListNodes(hashtable_t* allNodes_table);

  /* Checks if a vaild path found (wrt my_map and constraints)
     Note -- constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint).
     Returns true/false.
  */
  inline bool isConstrained(int curr_id, int next_id, int next_timestep, const vector< list< pair<int, int> > >* cons);

  /* Updates the path datamember (vector<int>).
     After update it will contain the sequence of locations found from the goal to the start.
  */
  void updatePath(Node* goal);  // $$$ make inline?

  /* Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
     Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
   */
  int numOfConflictsForStep(int curr_id, int next_id, int next_timestep, bool* res_table, int max_plan_len);

  /* Iterate over OPEN and adds to FOCAL all nodes with: 1) f-val > old_min_f_val ; and 2) f-val * f_weight < new_lower_bound.
   */
  void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);

  /* Returns true if a collision free path found (with cost up to f_weight * f-min) while
     minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
  */
  bool findPath(double f_weight, const vector < list< pair<int, int> > >* constraints, bool* res_table, size_t max_plan_len);

  ~SingleAgentECBS();
};

#endif
