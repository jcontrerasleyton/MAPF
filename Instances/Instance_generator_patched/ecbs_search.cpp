#include "ecbs_search.h"
#include <exception>
#include <iostream>
#include <utility>
#include <list>
#include <vector>
#include <tuple>
#include <ctime>
#include <climits>
#include <algorithm>
#include <random>
#include <iterator>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/graphviz.hpp>
#include <boost/graph/connected_components.hpp>
#include <iomanip>


void ECBSSearch::printPaths() {
  for (size_t i = 0; i < paths.size(); i++) {
    cout << "AGENT " << i << " Path: ";
    int cont = 0;
    for (vector<int>::const_iterator it = paths[i]->begin(); it != paths[i]->end(); ++it){
      std::cout << *it << ' ';
      cont++;
    }
    
    cout << " -- size: " << cont << endl;
  }
}

void ECBSSearch::savePaths(ofstream& res_f) {
  for (size_t i = 0; i < paths.size(); i++) {
    res_f << "AGENT " << i << " Path: ";
    for (vector<int>::const_iterator it = paths[i]->begin(); it != paths[i]->end(); ++it)
      res_f << ml->row_coordinate(*it) << ',' << ml->col_coordinate(*it) << " ; ";
    res_f << endl;
  }
}


void ECBSSearch::printResTable(bool* res_table, size_t max_plan_len) {
  cout << "MAP_SIZE=" << map_size << " ; MAX_PLAN_LEN=" << max_plan_len << endl;
  for (size_t t = 0; t < max_plan_len; t++) {
    for (size_t id = 0; id < (size_t)map_size; id++) {
      if ( res_table[id + t*map_size] == false )
        cout << '_';
      else
        cout << '*';
    }
    cout << endl;
  }
}


inline void ECBSSearch::releaseClosedListNodes() {
  hashtable_t::iterator it;
  for (it=allNodes_table.begin(); it != allNodes_table.end(); it++) {
    delete ( (*it).first );  // should it be .second?
  }
}


// computes g_val based on current paths (WRONG!)
inline double ECBSSearch::compute_g_val() {
  double retVal = 0;
  for (int i = 0; i < num_of_agents; i++)
    retVal += paths[i]->size();
  return retVal;
}


// computes High-Level lower-bound based
inline double ECBSSearch::compute_hl_lower_bound() {
  double retVal = 0;
  for (int i = 0; i < num_of_agents; i++)
    retVal += ll_min_f_vals[i];
  return retVal;
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ECBSSearch::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) {
  for (ECBSNode* n : open_list) {
    if ( n->sum_min_f_vals > old_lower_bound &&
         n->sum_min_f_vals <= new_lower_bound )
      n->focal_handle = focal_list.push(n);
  }
}


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ECBSSearch::updatePaths(ECBSNode* curr, ECBSNode* root_node) {
  paths = paths_found_initially;
  ll_min_f_vals = ll_min_f_vals_found_initially;
  paths_costs = paths_costs_found_initially;
  vector<bool> updated(num_of_agents, false);  // initialized for false
  /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
   * because younger nodes take into account ancesstors' nodes constraints. */
  while ( curr != root_node ) {
    if (updated[curr->agent_id] == false) {
      paths[curr->agent_id] = &(curr->path);
      ll_min_f_vals[curr->agent_id] = curr->ll_min_f_val;
      paths_costs[curr->agent_id] = curr->path_cost;
      updated[curr->agent_id] = true;
    }
    curr = curr->parent;
  }
}


// Used in the GUI
void ECBSSearch::updatePathsForExpTime(int t_exp) {
  if (t_exp > (int)HL_num_expanded || t_exp < 0)
    return;  // do nothing if there's no high-level node for the specified time_expanded

  ECBSNode* t_exp_node = NULL;
  for (hashtable_t::iterator it=allNodes_table.begin(); it != allNodes_table.end(); it++)
    if ( ((*it).second)->time_expanded == t_exp )
      t_exp_node = (*it).second;

  updatePaths(t_exp_node, dummy_start);
  //  printPaths();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them.
// returns true only if such a path exists (otherwise false and path remain empty).
inline bool ECBSSearch::updateECBSNode(ECBSNode* leaf_node, ECBSNode* root_node) {
  // extract all constraints on leaf_node->agent_id
  list < tuple<int, int, int> > constraints;  // each constraint is <L1,L2,T>
  int agent_id = leaf_node->agent_id;
  //  cout << " update ECBS node for agent:" << agent_id << endl;
  ECBSNode* curr = leaf_node;
  //  cout << "  Find all constraints on him:" << endl;
  while (curr != root_node) {
    if (curr->agent_id == agent_id) {
      constraints.push_front(curr->constraint);
      //      cout << "   L1:" << get<0>(curr->constraint) << " ; L2:" << get<1>(curr->constraint) << " ; T:" << get<2>(curr->constraint) << endl;
    }
    curr = curr->parent;
  }
  // cout << "  OVERALL #CONS:" << constraints.size() << endl;

  // calc constraints' max_timestep
  int max_timestep = -1;
  for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++)
    if ( get<2>(*it) > max_timestep )
      max_timestep = get<2>(*it);
  // cout << "  Latest constraint's timestep:" << max_timestep << endl;

  // initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
  //  cout << "  Creating a list of constraints (per timestep):" << endl;
  vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > > ( max_timestep+1, list< pair<int, int> > () );
  for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) {
    //    cout << "   PUSHING a constraint for time:" << get<2>(*it) << " ; (constraint is [" << get<0>(*it) << "," << get<1>(*it) << "])" << endl;
    cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));
  }

  // build reservation table
  size_t max_plan_len = getPathsMaxLength();
  bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
  updateReservationTable(res_table, max_plan_len, agent_id);

  //  printResTable(res_table, max_plan_len);

  // find a path w.r.t cons_vec (and prioretize by res_table).
  bool foundSol = search_engines[agent_id]->findPath(focal_w, cons_vec, res_table, max_plan_len);
  LL_num_expanded += search_engines[agent_id]->num_expanded;
  LL_num_generated += search_engines[agent_id]->num_generated;

#ifndef NDEBUG
  cout << "Run search for AG" << agent_id << " ; found solution? " << std::boolalpha << foundSol;
#endif
  // update leaf's path to the one found and its low-level search's min f-val
  if (foundSol) {
    leaf_node->path = vector<int>(*(search_engines[agent_id]->getPath()));
    leaf_node->ll_min_f_val = search_engines[agent_id]->min_f_val;
    leaf_node->path_cost = search_engines[agent_id]->path_cost;
#ifndef NDEBUG
    cout << " ; path-cost=" << (leaf_node->path_cost) << " ; for which min-f-val=" << leaf_node->ll_min_f_val << " ; The path is:";
    for (vector<int>::iterator it = leaf_node->path.begin(); it != leaf_node->path.end(); it++)
      cout << *it << " ";
#endif
  }
  //  cout << endl;

  // release memory allocated here and return
  delete (cons_vec);
  delete[] res_table;
  return foundSol;
}
////////////////////////////////////////////////////////////////////////////////

/*
  return agent_id's location for the given timestep
  Note -- if timestep is longer than its plan length,
          then the location remains the same as its last cell)
 */
inline int ECBSSearch::getAgentLocation(int agent_id, size_t timestep) {
  // if last timestep > plan length, agent remains in its last location
  if (timestep >= paths[agent_id]->size())
    return paths[agent_id]->at(paths[agent_id]->size()-1);
  // otherwise, return its location for that timestep
  return paths[agent_id]->at(timestep);
}

/*
  return true iff agent1 and agent2 switched locations at timestep [t,t+1]
 */
inline bool ECBSSearch::switchedLocations(int agent1_id, int agent2_id, size_t timestep) {
  // if both agents at their goal, they are done moving (cannot switch places)
  if ( timestep >= paths[agent1_id]->size() && timestep >= paths[agent2_id]->size() )
    return false;
  if ( getAgentLocation(agent1_id, timestep) == getAgentLocation(agent2_id, timestep+1) &&
       getAgentLocation(agent1_id, timestep+1) == getAgentLocation(agent2_id, timestep) )
    return true;
  return false;
}

/*
  Emulate agents' paths and returns a vector of collisions
  Note - a collision is a tuple of <int agent1_id, agent2_id, int location1, int location2, int timestep>).
  Note - the tuple's location_2=-1 for vertex collision.
 */
vector< tuple<int, int, int, int, int> >* ECBSSearch::extractCollisions() {
  vector< tuple<int, int, int, int, int> >* cons_found = new vector< tuple<int, int, int, int, int> >();
  earliest_conflict = make_tuple(-1, -1, -1, -1, INT_MAX);
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
        if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ) {
          cons_found->push_back(make_tuple(a1,
                                           a2,
                                           getAgentLocation(a1, timestep),
                                           -1,  // vertex collision (hence loc2=-1)
                                           timestep) );
          if ((int)timestep < std::get<4>(earliest_conflict))
            earliest_conflict = make_tuple(a1, a2, getAgentLocation(a1, timestep), -1, timestep);
        }
        if ( switchedLocations(a1, a2, timestep) ) {
          cons_found->push_back(make_tuple(a1,
                                           a2,
                                           getAgentLocation(a1, timestep),
                                           getAgentLocation(a2, timestep),
                                           timestep) );
          if ((int)timestep < std::get<4>(earliest_conflict))
            earliest_conflict = make_tuple(a1, a2, getAgentLocation(a1, timestep), getAgentLocation(a2, timestep), timestep);
        }
      }
    }
  }
  return cons_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Returns the maximal path length (among all agent)
size_t ECBSSearch::getPathsMaxLength() {
  size_t retVal = 0;
  for (int ag = 0; ag < num_of_agents; ag++)
    if ( paths[ag] != NULL && paths[ag]->size() > retVal )
      retVal = paths[ag]->size();
  return retVal;
}

// Generates a boolean reservation table for paths (cube of map_size*max_timestep).
// This is used by the low-level ECBS to count possible collisions efficiently
// Note -- we do not include the agent for which we are about to plan for
void ECBSSearch::updateReservationTable(bool* res_table, size_t max_path_len, int exclude_agent) {
  for (int ag = 0; ag < num_of_agents; ag++) {
    if (ag != exclude_agent && paths[ag] != NULL) {
      for (size_t timestep = 0; timestep < max_path_len; timestep++) {
        int id = getAgentLocation(ag, timestep);
        res_table[id + timestep*map_size] = true;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// h_lir (worked well in terms of trade off between time to compute and informativeness. Also, might be admissible...)
// it's basically a quick approximation of the vertex cover
int ECBSSearch::computeNumOfCollidingAgents() {
  int retVal = 0;
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
        //        cout << "   A1:" << getAgentLocation(a1, timestep) << ", A2:" <<  getAgentLocation(a2, timestep) << ", T:" << timestep;
        if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ||
             switchedLocations(a1, a2, timestep) ) {
          //          cout << "  A" << a1 << " -- A" << a2 << endl;
          retVal++;
          // break to the outer (a1) loop
          timestep = max_path_length;
          a2 = num_of_agents;
        }
      }
    }
  }
  //  cout << "   *-*-* h_lir = " << retVal << endl;
  return retVal;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compute the number of agents that has at least one conflict (h_2 in ECBS's paper)
int ECBSSearch::compute_h2() {
  int retVal = 0;
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = 0; a2 < num_of_agents; a2++) {
      if (a1 != a2) {
        size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
        for (size_t timestep = 0; timestep < max_path_length; timestep++) {
          //        cout << "   A1:" << getAgentLocation(a1, timestep) << ", A2:" <<  getAgentLocation(a2, timestep) << ", T:" << timestep;
          if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ||
               switchedLocations(a1, a2, timestep) ) {
            //          cout << "  A" << a1 << " -- A" << a2 << endl;
            retVal++;
            timestep = max_path_length;  // break this loop
            a2 = num_of_agents;  // break to the outer (a1) loop
          }
        }
      }
    }
  }
  //  cout << "   *-*-* h_2 = " << retVal << endl;
  return retVal;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compute the number of pairs of agents colliding (h_3 in ECBS's paper)
int ECBSSearch::compute_h3() {
  int retVal = 0;
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
        //        cout << "   A1:" << getAgentLocation(a1, timestep) << ", A2:" <<  getAgentLocation(a2, timestep) << ", T:" << timestep;
        if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ||
             switchedLocations(a1, a2, timestep) ) {
          //          cout << "  A" << a1 << " -- A" << a2 << endl;
          retVal++;
          timestep = max_path_length;  // break this loop
        }
      }
    }
  }
  //  cout << "   *-*-* h_3 = " << retVal << endl;
  return retVal;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// compute the tree-width
////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ECBSSearch::compute_treewidth(vector< tuple<int, int, int, int, int> >* collision_vec) {
  // sort according to agents (might not be needed)
  std::sort(collision_vec->begin(),
            collision_vec->end(),
            [](tuple<int, int, int, int, int> const &t1, tuple<int, int, int, int, int> const &t2) {
              if (get<0>(t1) == get<0>(t2))
                return get<1>(t1) < get<1>(t2);
              return get<0>(t1) < get<0>(t2);
            }
            );
  // build agent map (treed assumes nodes are 0 to N)
  int ag_counter = 0;
  vector<int> ag_map(num_of_agents, -1);
  for (size_t i = 0; i < collision_vec->size(); i++) {
    int ag1 = get<0>(collision_vec->at(i));
    int ag2 = get<1>(collision_vec->at(i));
    if ( ag_map[ag1] == -1 ) {
      ag_map[ag1] = ag_counter;
      ag_counter++;
    }
    if ( ag_map[ag2] == -1 ) {
      ag_map[ag2] = ag_counter;
      ag_counter++;
    }
  }
  // export the graph to treed (note -- it seems like the same edges is OK, so no need to distinguish two
  //    different timesteps constraining the same pair of agents...)
  int numVars = ag_counter;
  int numCons = collision_vec->size();
  vector<vector<int> > hyperedges(numCons);
  for (size_t i = 0; i < collision_vec->size(); i++) {
    int ag1 = get<0>(collision_vec->at(i));
    int ag2 = get<1>(collision_vec->at(i));
    hyperedges[i].push_back(ag_map[ag1]);
    hyperedges[i].push_back(ag_map[ag2]);
  }
  GraphAlgoWrapper tww;
  return tww.getTW(numVars, numCons, hyperedges);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// compute the minimum vertex cover
////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ECBSSearch::compute_vertexcover(vector< tuple<int, int, int, int, int> >* collision_vec) {
  // sort according to agents (might not be needed)
  std::sort(collision_vec->begin(),
            collision_vec->end(),
            [](tuple<int, int, int, int, int> const &t1, tuple<int, int, int, int, int> const &t2) {
              if (get<0>(t1) == get<0>(t2))
                return get<1>(t1) < get<1>(t2);
              return get<0>(t1) < get<0>(t2);
            }
            );
  // build agent map (treed assumes nodes are 0 to N)
  // NOTE -- DIMACS file format requires the first node to start from 1!!
  // (if we ever want to read back results, remember to do the right conversion because we start from 0!!)
  int ag_counter = 1;
  vector<int> ag_map(num_of_agents, -1);
  for (size_t i = 0; i < collision_vec->size(); i++) {
    int ag1 = get<0>(collision_vec->at(i));
    int ag2 = get<1>(collision_vec->at(i));
    if ( ag_map[ag1] == -1 ) {
      ag_map[ag1] = ag_counter;
      ag_counter++;
    }
    if ( ag_map[ag2] == -1 ) {
      ag_map[ag2] = ag_counter;
      ag_counter++;
    }
  }
  // export the graph to treed (note -- it seems like the same edges is OK, so no need to distinguish two
  //    different timesteps constraining the same pair of agents...)
  int numVars = ag_counter;
  vector< pair<int, int> > aig_edges(0);
  for (size_t i = 0; i < collision_vec->size(); i++) {
    int ag1 = get<0>(collision_vec->at(i));
    int ag2 = get<1>(collision_vec->at(i));
    aig_edges.push_back( make_pair(ag_map[ag1], ag_map[ag2]) );
  }
  GraphAlgoWrapper mvc;
  return mvc.getMVC(numVars, aig_edges);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// compute the number of independent sets (via Boost's BGL)
// load the graph from an Agents Interaction Graph file
int ECBSSearch::compute_connected_components(string dot_filename) {
/*
  using namespace boost;
  adjacency_list< vecS, vecS, undirectedS > g(0);
  dynamic_properties dp(ignore_other_properties);
  ifstream fin(dot_filename);
  read_graphviz(fin, g, dp);
  fin.close();
  //  cout << endl << "|V|=" << num_vertices(g) << " ; |E|=" << num_edges(g) << endl;
  vector<int> component(num_vertices(g));
  int retVal = connected_components(g, &component[0]);
  //  for (i = 0; i != component.size(); ++i)
  //    cout << "Vertex " << i <<" is in component " << component[i] << endl;
  return retVal;
*/
return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////
// compute the size of the 1-cutset set
////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ECBSSearch::exportAGItoDOT(string dot_filename, vector< tuple<int, int, int, int, int> >* collision_vec, string prompt_line) {
  // note 1 -- maybe not needed (its built sorted!)
  // sort collisions according to first agents (break ties towards the second agent)
  //    note 2 -- check why auto const &t1 doesn't work...
  std::sort(collision_vec->begin(),
            collision_vec->end(),
            [](tuple<int, int, int, int, int> const &t1, tuple<int, int, int, int, int> const &t2) {
              if (get<0>(t1) == get<0>(t2))
                return get<1>(t1) < get<1>(t2);
              return get<0>(t1) < get<0>(t2);
            }
            );
  //  for (vector< tuple<int,int,int,int,int> >::const_iterator it = collision_vec->begin(); it != collision_vec->end(); it++)
  // cout << "A1:" << get<0>(*it) << " ; A2:" << get<1>(*it) << " ; L1:" << get<2>(*it) << " ; L2:" << get<3>(*it) << " ; T:" << get<4>(*it) << endl;
  ofstream myfile;
  myfile.open(dot_filename);
  myfile << "graph {" << endl;
  if (collision_vec->size() > 0)
    myfile << "  " << to_string(get<0>(collision_vec->at(0))) << " -- "
           << to_string(get<1>(collision_vec->at(0))) << " [label=\" "
           << "t" << to_string(get<4>(collision_vec->at(0)));
  for (size_t i = 1; i < collision_vec->size(); i++) {
    int const ag1 = get<0>(collision_vec->at(i));
    int const ag2 = get<1>(collision_vec->at(i));
    int const t = get<4>(collision_vec->at(i));
    int const prev_ag1 = get<0>(collision_vec->at(i-1));
    int const prev_ag2 = get<1>(collision_vec->at(i-1));
    if (ag1 == prev_ag1 && ag2 == prev_ag2) {
      myfile << ", t" << to_string(t);  // add current conflict time to an existing edge
    } else {
      myfile << "\"]" << endl;  // close the previous edge
      myfile << "  " << to_string(ag1) << " -- "
             << to_string(ag2) << " [label=\" "
             << "t" << to_string(t);
    }
    
  }
  myfile << "\"]" << endl;
  // build agent map (treed assumes nodes are 0 to N)
  int ag_counter = 0;
  vector<int> ag_map(num_of_agents, -1);
  for (size_t i = 0; i < collision_vec->size(); i++) {
    int ag1 = get<0>(collision_vec->at(i));
    int ag2 = get<1>(collision_vec->at(i));
    if ( ag_map[ag1] == -1 ) {
      ag_map[ag1] = ag_counter;
      ag_counter++;
    }
    if ( ag_map[ag2] == -1 ) {
      ag_map[ag2] = ag_counter;
      ag_counter++;
    }
  }
  for (int i = 0; i < num_of_agents; i++) {
    if (ag_map[i] != -1) {
      myfile << i << " [label=\"" << i << "\n"
             << search_engines[i]->num_expanded << "\n"
             << std::setprecision(3) << search_engines[i]->path_cost / search_engines[i]->min_f_val << "\n"
             << search_engines[i]->path_cost << "\"]" << endl;
    }
  }
  myfile << "  label=\"Map: " << ml->getMapName()
         << "\n Agents: " << al->getAgentsName()
         << "\n Highway: " << egr->getEGRName()
         << "\n w_focal=" << focal_w
         << " ; w_hwy=" << hwy_w
         << "\n Agents Order: {";
  std::copy(agents_order.begin(), agents_order.end(), std::ostream_iterator<int>(myfile, ","));
  myfile << "}\n";
  myfile << prompt_line << "\"" << endl;
  myfile << "}" << endl;
  myfile.close();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ECBSSearch::addLabeltoDOT(string dot_filename, string prompt_line) {
  fstream myfile;
  myfile.open(dot_filename);
  myfile.seekg(-4,ios_base::end);  // goes to EOF and then back a few characters to continue the label
  myfile << "\n " << prompt_line << "\"" << endl << "}" << endl;
  myfile.close();
  return;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////
ECBSSearch::ECBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, double f_w, bool tweak_g_val, int rrr_it, bool rand_succ_gen) :
  ml(&ml), al(&al), egr(&egr), rand_succ_gen(rand_succ_gen) {
  focal_w = f_w;
  hwy_w = e_w;
  HL_num_expanded = 0;
  HL_num_generated = 0;
  LL_num_expanded = 0;
  LL_num_generated = 0;
  num_of_agents = al.num_of_agents;
  map_size = ml.rows*ml.cols;
  solution_found = false;
  solution_cost = -1;
  ll_min_f_vals = vector <double> (num_of_agents);
  paths_costs = vector <double> (num_of_agents);
  ll_min_f_vals_found_initially = vector <double> (num_of_agents);
  paths_costs_found_initially = vector <double> (num_of_agents);
  search_engines = vector < SingleAgentECBS* > (num_of_agents);
  for (int i = 0; i < num_of_agents; i++) {
    int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
    int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
    ComputeHeuristic ch(goal_loc, ml.get_map(), ml.rows*ml.cols, ml.actions_offset, e_w, &egr);
    search_engines[i] = new SingleAgentECBS(init_loc, goal_loc,
                                            ch.getHVals(),
                                            ml.get_map(), ml.rows*ml.cols, ml.actions_offset,
                                            &egr,
                                            e_w,
                                            tweak_g_val,
					    rrr_it,
					    rand_succ_gen);
  }

  // initialize allNodes_table (hash table)
  empty_node = new ECBSNode();
  empty_node->time_generated = -2; empty_node->agent_id = -2;
  deleted_node = new ECBSNode();
  deleted_node->time_generated = -3; deleted_node->agent_id = -3;
  allNodes_table.set_empty_key(empty_node);
  allNodes_table.set_deleted_key(deleted_node);

  // initialize all initial paths to NULL
  paths_found_initially.resize(num_of_agents);
  for (int ag=0; ag < num_of_agents; ag++)
    paths_found_initially[ag] = NULL;

  // create the sequential (0,1,2,...,num_of_agents-1) agents ordering
  agents_order = vector<int>(num_of_agents);
  for (int i = 0; i < num_of_agents; i++)
    agents_order.at(i) = i;
  // permute it if needed for RRR
  if (rrr_it != 0) {  // create (a random, where the seed is rrr_it) permutation
    // note -- the following replace the constant seed with a random one
    //    std::random_device rd;
    //    std::mt19937 g(rd());
    std::mt19937 g(rrr_it);
    std::shuffle(agents_order.begin(), agents_order.end(), g);
  }
  /*
  // print agents order
  cout << endl << endl << "AGENTS ORDER: ";
  std::copy(agents_order.begin(), agents_order.end(), std::ostream_iterator<int>(std::cout, ", "));
  cout << endl << endl;
  */

  // initialize paths_found_initially
  for (int i : agents_order) {
    //    cout << "Computing initial path for agent " << i << endl; fflush(stdout);
    paths = paths_found_initially;
    size_t max_plan_len = getPathsMaxLength();
    bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
    updateReservationTable(res_table, max_plan_len, i);
    //    cout << "*** CALCULATING INIT PATH FOR AGENT " << i << ". Reservation Table[MAP_SIZE x MAX_PLAN_LEN]: " << endl;
    //    printResTable(res_table, max_plan_len);
    if ( search_engines[i]->findPath ( f_w, NULL, res_table, max_plan_len ) == false)
      cout << "NO SOLUTION EXISTS";
    paths_found_initially[i] = new vector<int> (*(search_engines[i]->getPath()));
    ll_min_f_vals_found_initially[i] = search_engines[i]->min_f_val;
    paths_costs_found_initially[i] = search_engines[i]->path_cost;
    LL_num_expanded += search_engines[i]->num_expanded;
    LL_num_generated += search_engines[i]->num_generated;
    delete[] res_table;
    //    cout << endl;
  }

  paths = paths_found_initially;
  ll_min_f_vals = ll_min_f_vals_found_initially;
  paths_costs = paths_costs_found_initially;

  // generate dummy start and update data structures
  dummy_start = new ECBSNode();
  dummy_start->agent_id = -1;
  dummy_start->g_val = 0;
  for (int i = 0; i < num_of_agents; i++)
    dummy_start->g_val += paths_costs[i];
  dummy_start->ll_min_f_val = 0;
  dummy_start->sum_min_f_vals = compute_hl_lower_bound();
  dummy_start->open_handle = open_list.push(dummy_start);
  dummy_start->focal_handle = focal_list.push(dummy_start);
  HL_num_generated++;
  dummy_start->time_generated = HL_num_generated;
  allNodes_table[dummy_start] = dummy_start;

  min_sum_f_vals = dummy_start->sum_min_f_vals;
  focal_list_threshold = focal_w * dummy_start->sum_min_f_vals;

  //  cout << "Paths in START (high-level) node:" << endl;
  //  printPaths();
  vector< tuple<int, int, int, int, int> >* collision_vec = extractCollisions();
  // generate agent-interaction-graph, its tree-decomposition and export them to .dot file
  dot_filename = (ml.map_filename).substr(0, ml.map_filename.find(".")) + "_"
    + (al.agents_filename).substr(0, al.agents_filename.find(".")) + "_"
    + (egr.egr_filename).substr(0, egr.egr_filename.find(".")) + "_"
    + "hw_" + to_string(hwy_w) + "_"
    + "fw_" + to_string(focal_w) + "_"
    + to_string(rrr_it) + "_ECBS.dot";
  std::replace( dot_filename.begin(), dot_filename.end(), '/', '_');
  //  exportAGItoDOT(dot_filename, collision_vec, "");
  /*
  string root_statistics = "ROOT SUM-MIN-F-VALS:" + to_string(dummy_start->sum_min_f_vals)
    + " ; ROOT SUM-COST:" + to_string(dummy_start->g_val)
    + " ; ROOT NUM-OF-CONFLICTS:" + to_string(collision_vec->size())
    + " ; ROOT h_lir:" + to_string(computeNumOfCollidingAgents())
    + " ; ROOT h_2:" + to_string(compute_h2())
    + " ; ROOT h_3:" + to_string(compute_h3())
    + " ; ROOT CC:"  + to_string(compute_connected_components(dot_filename))
    + " ; ROOT TW:" + to_string(compute_treewidth(collision_vec))
    + " ; ROOT MVC:" + to_string(compute_vertexcover(collision_vec));
  */
  /*
  cout << dummy_start->sum_min_f_vals << " ; "
       << dummy_start->g_val << " ; "
       << collision_vec->size() << " ; "
       << computeNumOfCollidingAgents() << " ; "
       << compute_h2() << " ; "
       << compute_h3() << " ; "
       << compute_connected_components(dot_filename) << " ; "
       << compute_treewidth(collision_vec) << " ; "
       << compute_vertexcover(collision_vec) << " ; ;";
  */
  //  fflush(stdout);
  //  exportAGItoDOT(dot_filename, collision_vec, root_statistics);
  delete(collision_vec);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ECBSSearch::runECBSSearch() {
  // set timer
  std::clock_t start;
  double duration;
  start = std::clock();

  // start is already in the open_list
  while ( !focal_list.empty() && !solution_found ) {
    // break after 5 min
    duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    if (duration > time_limit_cutoff) {  // timeout after 5 minutes
      cout << " -1 ; " << min_sum_f_vals << " ; "                                // -1 cost means failed
           << HL_num_expanded << " ; " << HL_num_generated << " ; "
           << LL_num_expanded << " ; " << LL_num_generated << " ; " << time_limit_cutoff << endl;
//      addLabeltoDOT(dot_filename, "FAILED (300s TIMEOUT)");
      return false;
    }

    ECBSNode* curr = focal_list.top();
    focal_list.pop();
    open_list.erase(curr->open_handle);
    HL_num_expanded++;
    curr->time_expanded = HL_num_expanded;
    //    cout << "Expanding: (" << curr << ")" << *curr << " at time:" << HL_num_expanded << endl;

    // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
    updatePaths(curr, dummy_start);
    //    printPaths();

    vector< tuple<int, int, int, int, int> >* collision_vec = extractCollisions();  // check for collisions on updated paths
#ifndef NDEBUG
    cout << endl << "****** Expanded #" << curr->time_expanded << " with cost " << curr->g_val << " and # Collisions " << collision_vec->size() << " and |FOCAL|=" << focal_list.size() << " and focal-threshold=" << focal_list_threshold << endl;
#endif
    /*    
    cout << "Collision found in the expanded node's paths:" << endl;
    for (vector< tuple<int,int,int,int,int> >::const_iterator it = collision_vec->begin(); it != collision_vec->end(); it++)
      cout << "   A1:" << get<0>(*it) << " ; A2:" << get<1>(*it) << " ; L1:" << get<2>(*it) << " ; L2:" << get<3>(*it) << " ; T:" << get<4>(*it) << endl;
    cout << "Overall Col_Vec.size=" << collision_vec->size() << endl;
     */

    if ( collision_vec->size() == 0 ) {  // found a solution (and finish the while look)
      solution_found = true;
      solution_cost = curr->g_val;
    } else {  // generate the two successors that resolve one of the conflicts
      int agent1_id, agent2_id, location1, location2, timestep;
      tie(agent1_id, agent2_id, location1, location2, timestep) = earliest_conflict;  // choose differently? (used to be collision_vec->at(0))
#ifndef NDEBUG
      cout << "   Earliest collision -- A1:" << agent1_id << " ; A2: " << agent2_id
	   << " ; L1:" << location1 << " ; L2:" << location2 << " ; T:" << timestep << endl;
#endif
      ECBSNode* n1 = new ECBSNode();
      ECBSNode* n2 = new ECBSNode();
      n1->agent_id = agent1_id;
      n2->agent_id = agent2_id;
      if (location2 == -1) {  // generate vertex constraint
        n1->constraint = make_tuple(location1, -1, timestep);
        n2->constraint = make_tuple(location1, -1, timestep);
      } else {  // generate edge constraint
        n1->constraint = make_tuple(location1, location2, timestep);
        n2->constraint = make_tuple(location2, location1, timestep);
      }
      n1->parent = curr;
      n2->parent = curr;
      //      cout << "*** Before solving, " << endl << *n1;
      // find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them. Also updates n1's g_val
      if ( updateECBSNode(n1, dummy_start) == true ) {
        // new g_val equals old g_val plus the new path length found for the agent minus its old path length
        n1->g_val = curr->g_val - paths_costs[n1->agent_id] + n1->path_cost;
        // update n1's path for computing the num of colliding agents
        vector<int>* temp_old_path = paths[n1->agent_id];
        paths[n1->agent_id] = &(n1->path);
        n1->h_val = computeNumOfCollidingAgents();
        paths[n1->agent_id] = temp_old_path;  // restore the old path (for n2)
        // update lower bounds and handles
        n1->sum_min_f_vals = curr->sum_min_f_vals - ll_min_f_vals[n1->agent_id] + n1->ll_min_f_val;
        n1->open_handle = open_list.push(n1);
        HL_num_generated++;
        n1->time_generated = HL_num_generated;
        if ( n1->sum_min_f_vals <= focal_list_threshold )
          n1->focal_handle = focal_list.push(n1);
        allNodes_table[n1] = n1;
#ifndef NDEBUG
	cout << endl << "   First node generated for A" << n1->agent_id << ": g-val=" << n1->g_val << " ; h-val=" << n1->h_val << " ; LB=" << n1->sum_min_f_vals << endl;
#endif
      } else {
        delete (n1);
      }
      // same for n2
      //      cout << "*** Before solving, " << endl << *n2;
      if ( updateECBSNode(n2, dummy_start) == true ) {
        n2->g_val = curr->g_val - paths_costs[n2->agent_id] + n2->path_cost;
        vector<int>* temp_old_path = paths[n2->agent_id];
        paths[n2->agent_id] = &(n2->path);
        n2->h_val = computeNumOfCollidingAgents();
        paths[n2->agent_id] = temp_old_path;
        n2->sum_min_f_vals = curr->sum_min_f_vals - ll_min_f_vals[n2->agent_id] + n2->ll_min_f_val;
        n2->open_handle = open_list.push(n2);
        HL_num_generated++;
        n2->time_generated = HL_num_generated;
        if ( n2->sum_min_f_vals <= focal_list_threshold )
          n2->focal_handle = focal_list.push(n2);
        allNodes_table[n2] = n2;
#ifndef NDEBUG        
	cout << endl << "   Second node generated for A" << n2->agent_id << ": g-val=" << n2->g_val << " ; h-val=" << n2->h_val << " ; LB=" << n2->sum_min_f_vals << endl;
#endif
      } else {
        delete (n2);
      }
      //            cout << "It has found the following paths:" << endl;
      //            printPaths();
      //            cout << "Focal threshold: (before) " << focal_list_threshold;
      if (open_list.size() == 0) {
        solution_found = false;
        break;
      }
      ECBSNode* open_head = open_list.top();
    if ( open_head->sum_min_f_vals > min_sum_f_vals ) {
#ifndef NDEBUG
	cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
#endif
        min_sum_f_vals = open_head->sum_min_f_vals;
        double new_focal_list_threshold = min_sum_f_vals * focal_w;
        updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
        focal_list_threshold = new_focal_list_threshold;
	// cout << focal_list.size() << endl;
      }
      //            cout << " ; (after) " << focal_list_threshold << endl << endl;
    }  // end generating successors
    delete (collision_vec);
  }  // end of while loop
  
  // get time
  duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  
  cout << solution_cost << " ; " << min_sum_f_vals << " ; " <<
      HL_num_expanded << " ; " << HL_num_generated << " ; " <<
      LL_num_expanded << " ; " << LL_num_generated << " ; " << duration << endl;
    //    printPaths();
  addLabeltoDOT(dot_filename,
                "SOL-COST: " + to_string(solution_cost) +
                " ; SUM-MIN-F-VALS: " + to_string(min_sum_f_vals) +
                " ; HL-EXP: " + to_string(HL_num_expanded) +
                " ; HL-GEN: " + to_string(HL_num_generated) +
                " ; LL-EXP: " + to_string(LL_num_expanded) + 
                " ; LL-GEN: " + to_string(LL_num_generated) +
                " ; TIME: " + to_string(duration)
                );

  return solution_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ECBSSearch::~ECBSSearch() {
  for (size_t i = 0; i < search_engines.size(); i++)
    delete (search_engines[i]);
  for (size_t i = 0; i < paths_found_initially.size(); i++)
    delete (paths_found_initially[i]);
  //  for (size_t i=0; i<paths.size(); i++)
  //    delete (paths[i]);
  releaseClosedListNodes();
  delete (empty_node);
  delete (deleted_node);
}
