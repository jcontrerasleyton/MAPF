// CBS Search (High-level)
#ifndef CBSSEARCH_H
#define CBSSEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include <cstring>
#include <climits>
#include <tuple>
#include <string>
#include <vector>
#include <list>
#include "map_loader.h"
#include "agents_loader.h"
#include "compute_heuristic.h"
#include "egraph_reader.h"
#include "single_agent_search.h"
#include "cbs_node.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using std::cout;

class CBSSearch {
 public:
  typedef boost::heap::fibonacci_heap< CBSNode* , boost::heap::compare<CBSNode::compare_node> > heap_open_t;
  typedef dense_hash_map<CBSNode*, CBSNode*, CBSNode::CBSNodeHasher, CBSNode::cbs_eqnode> hashtable_t;

  vector < vector<int>* > paths;  // agents paths (each entry [i] is a vector<int> which specify the locations on the path of agent i)
  vector < vector<int>* > paths_found_initially;  // contain initial paths found (that is, each with optimal policy)
  bool solution_found;
  double solution_cost;

  double cbs_time;

  list < CBSNode* > popped_nodes;  // used to clean the memory at the end
  CBSNode* dummy_start;
  vector <int> start_locations;
  vector <int> goal_locations;
  vector <bool> agents_in_goal;


  const bool* my_map;
  int map_size;
  int num_of_agents;
  const int* actions_offset;

  long long num_expanded = 0;
  long long num_generated = 0;
  long long low_expanded = 0;
  long long low_generated = 0;
  //int open_updated = 0;
  //int close_updated = 0;

  int w_window_limit = -1;

  heap_open_t heap;

  size_t time_limit_cutoff = 600;

  vector < SingleAgentSearch* > search_engines;  // used to find (single) agents' paths

  CBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, bool tweak_g_val = false, int w_limit = -1);
  void NextIteration();
  bool checkGoal();
  inline double compute_g_val(int num_of_agents);
  inline void updatePaths(CBSNode* curr , CBSNode* root_node);
  inline bool updateCBSNode(CBSNode* leaf_node, CBSNode* root_node);
  bool runCBSSearch();
  inline bool switchedLocations(int agent1_id, int agent2_id, size_t timestep);
  inline int getAgentLocation(int agent_id, size_t timestep);
  vector< tuple<int, int, int, int, int> >* extractCollisions(int num_of_agents);
  void printPaths();
  void updatePathsForExpTime(int t_exp);

  ~CBSSearch();
};

#endif
