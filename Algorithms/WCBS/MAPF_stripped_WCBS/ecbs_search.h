// ECBS Search (High-level)
#ifndef ECBSSEARCH_H
#define ECBSSEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include "../../../include/sparsehash/dense_hash_map"
#include <cstring>
#include <climits>
#include <tuple>
#include <string>
#include <vector>
#include <list>
#include <fstream>
#include "map_loader.h"
#include "agents_loader.h"
#include "compute_heuristic.h"
#include "egraph_reader.h"
#include "single_agent_ecbs.h"
#include "ecbs_node.h"
#include "map_loader.h"
#include "agents_loader.h"
#include "graph_algo_wrapper.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using std::cout;
using std::endl;
using google::dense_hash_map;

class ECBSSearch {
 public:

  const MapLoader* ml;
  const AgentsLoader* al;
  const EgraphReader* egr;

  double focal_w = 1.0;
  double hwy_w = 1.0;
  vector<int> agents_order;
  double focal_list_threshold;
  double min_sum_f_vals;

  const bool rand_succ_gen;

  typedef boost::heap::fibonacci_heap< ECBSNode* , boost::heap::compare<ECBSNode::compare_node> > heap_open_t;
  typedef boost::heap::fibonacci_heap< ECBSNode* , boost::heap::compare<ECBSNode::secondary_compare_node> > heap_focal_t;
  typedef dense_hash_map<ECBSNode*, ECBSNode*, ECBSNode::ECBSNodeHasher, ECBSNode::ecbs_eqnode> hashtable_t;

  vector < vector<int>* > paths;  // agents paths (each entry [i] is a vector<int> which specify the locations on the path of agent i)
  vector < vector<int>* > paths_found_initially;  // contain initial paths found
  bool solution_found;
  double solution_cost;

  ECBSNode* dummy_start;
  vector <int> start_locations;
  vector <int> goal_locations;

  const bool* my_map;
  int map_size;
  int num_of_agents;
  const int* actions_offset;

  uint64_t HL_num_expanded = 0;
  uint64_t HL_num_generated = 0;
  uint64_t LL_num_expanded = 0;
  uint64_t LL_num_generated = 0;

  heap_open_t open_list;
  heap_focal_t focal_list;
  hashtable_t allNodes_table;

  // used in hash table and would be deleted from the d'tor
  ECBSNode* empty_node;
  ECBSNode* deleted_node;

  vector < SingleAgentECBS* > search_engines;  // used to find (single) agents' paths
  vector <double> ll_min_f_vals_found_initially;  // contains initial ll_min_f_vals found
  vector <double> ll_min_f_vals;  // each entry [i] represent the lower bound found for agent[i]
  vector <double> paths_costs_found_initially;
  vector <double> paths_costs;

  tuple<int, int, int, int, int> earliest_conflict;  // saves the earliest conflict (updated in every call to extractCollisions()).

  string dot_filename;

  size_t time_limit_cutoff = 300;

  ECBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, double e_f, bool tweak_g_val = false, int rrr_it = 0, bool rand_succ_gen = false);
  inline double compute_g_val();
  inline double compute_hl_lower_bound();
  inline void updatePaths(ECBSNode* curr , ECBSNode* root_node);
  inline bool updateECBSNode(ECBSNode* leaf_node, ECBSNode* root_node);
  bool runECBSSearch();
  inline bool switchedLocations(int agent1_id, int agent2_id, size_t timestep);
  inline int getAgentLocation(int agent_id, size_t timestep);
  vector< tuple<int, int, int, int, int> >* extractCollisions();
  void printPaths();
  void savePaths(ofstream& res_f);
  void printResTable(bool* res_table, size_t max_plan_len);
  void updatePathsForExpTime(int t_exp);

  size_t getPathsMaxLength();
  void updateReservationTable(bool* res_table, size_t max_plan_len, int exclude_agent);

  void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);

  int computeNumOfCollidingAgents();
  int compute_h2();
  int compute_h3();
  int compute_connected_components(string dot_filename);
  int compute_treewidth(vector< tuple<int, int, int, int, int> >* collision_vec);
  int compute_vertexcover(vector< tuple<int, int, int, int, int> >* collision_vec);

  void exportAGItoDOT(string dot_filename, vector< tuple<int, int, int, int, int> >* collision_vec, string prompt_line);
  void addLabeltoDOT(string dot_filename, string prompt_line);

  inline void releaseClosedListNodes();

  ~ECBSSearch();
};

#endif
