//CBS_anytime

#include "cbs_search.h"
#include <exception>
#include <iostream>
#include <utility>
#include <list>
#include <vector>
#include <tuple>
#include <ctime>

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using std::cout;


void CBSSearch::printPaths() {
  int max_path = 0;
  for (size_t i = 0; i < (size_t)paths.size(); i++) {
    if((int)paths[i]->size() > (int)max_path) max_path = paths[i]->size();
  }

  cout << "\nT\t";
  for (size_t i = 0; i < (size_t)max_path; i++) {
    cout << i << '\t';
  }
  cout << endl;
  cout << "    ---------------------------------------------------------------------";
  cout << "-------------------------------------------------------------------------" << endl;

  for (size_t i = 0; i < paths.size(); i++) {
    cout << "A" << i << " |\t";
    for (vector<int>::const_iterator it = paths[i]->begin(); it != paths[i]->end(); ++it){
      std::cout << *it << '\t';
    }
    cout << endl << endl;
  }
  cout << endl;
}

// computes g_val based on current paths
inline double CBSSearch::compute_g_val(int num_of_agents) {
  double retVal = 0;
  for (int i = 0; i < num_of_agents; i++)
    retVal += paths[i]->size();
  return retVal;
}


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
inline void CBSSearch::updatePaths(CBSNode* curr, CBSNode* root_node) {
  paths = paths_found_initially;
  vector<bool> updated(paths.size());
  /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
   * because younger nodes take into account ancesstors' nodes constraints. */
  for (size_t i = 0; i < paths.size(); i++)
    updated.push_back(false);
  while ( curr != root_node ) {
    if (updated[curr->agent_id] == false) {
      paths[curr->agent_id] = &(curr->path);
      updated[curr->agent_id] = true;
    }
    curr = curr->parent;
  }
}

// Used in the GUI
void CBSSearch::updatePathsForExpTime(int t_exp) {
  if (t_exp > num_expanded || t_exp < 0)
    return;  // do nothing if there's no high-level node for the specified time_expanded

  CBSNode* t_exp_node = NULL;
  for ( list < CBSNode* >::iterator it = popped_nodes.begin(); it != popped_nodes.end() && t_exp_node == NULL; it++)
    if ( (*it)->time_expanded == t_exp )
      t_exp_node = *it;

  updatePaths(t_exp_node, dummy_start);
  printPaths(); 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them.
// returns true only if such a path exists (otherwise false and path remain empty).
inline bool CBSSearch::updateCBSNode(CBSNode* leaf_node, CBSNode* root_node) {
  // extract all constraints on leaf_node->agent_id
  list < tuple<int, int, int> > constraints;
  int agent_id = leaf_node->agent_id;
  CBSNode* curr = leaf_node;
  while (curr != root_node) {
    if (curr->agent_id == agent_id)
      constraints.push_front(curr->constraint);
    curr = curr->parent;
  }

  // calc max_timestep
  int max_timestep = -1;
  for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++)
    if ( get<2>(*it) > max_timestep )
      max_timestep = get<2>(*it);

  // initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
  vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > > ( max_timestep+1, list< pair<int, int> > () );
  for ( list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++)
    cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));

  // find a path w.r.t cons_vec
  if ( search_engines[agent_id]->findPath( cons_vec, w_window_limit ) == false )
    return false;

  // update leaf's path to the one found
  leaf_node->path = vector<int>(*(search_engines[agent_id]->getPath()));
  delete (cons_vec);
  return true;
}
////////////////////////////////////////////////////////////////////////////////

/*
  return agent_id's location for the given timestep
  Note -- if timestep is longer than its plan length,
          then the location remains the same as its last cell)
 */
inline int CBSSearch::getAgentLocation(int agent_id, size_t timestep) {
  // if last timestep > plan length, agent remains in its last location
  if (timestep >= paths[agent_id]->size())
    return paths[agent_id]->at(paths[agent_id]->size()-1);
  // otherwise, return its location for that timestep
  return paths[agent_id]->at(timestep);
}

/*
  return true iff agent1 and agent2 switched locations at timestep [t,t+1]
 */
inline bool CBSSearch::switchedLocations(int agent1_id, int agent2_id, size_t timestep) {
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
vector< tuple<int, int, int, int, int> >* CBSSearch::extractCollisions(int num_of_agents) {
  vector< tuple<int, int, int, int, int> >* cons_found = new vector< tuple<int, int, int, int, int> >();
  // check for vertex and edge collisions
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
        if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ) {
          //cout << "colision_same_time:\t" << "A" << a1 << " & A" << a2 << " in timestep " << timestep << endl;
          cons_found->push_back(make_tuple(a1,
                                           a2,
                                           getAgentLocation(a1, timestep),
                                           -1,  // vertex collision (hence loc2=-1)
                                           timestep) );
        }
        if ( switchedLocations(a1, a2, timestep) ) {
          //cout << "colision_switched:\t" << "A" << a1 << " & A" << a2 << " in timestep " << timestep << endl;
          cons_found->push_back(make_tuple(a1,
                                           a2,
                                           getAgentLocation(a1, timestep),
                                           getAgentLocation(a2, timestep),
                                           timestep) );
        }
      }
    }
  }
  //cout << "---------------------------------------------------------------------------------------->>" << endl;
  return cons_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CBSSearch::CBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, bool tweak_g_val, int w_limit) {
  w_window_limit = w_limit;
  cbs_time = 0;
  num_expanded = 0;
  num_generated = 0;
  low_expanded = 0;
  low_generated = 0;
  num_of_agents = al.num_of_agents;
  map_size = ml.rows*ml.cols;
  solution_found = false;
  solution_cost = 0;
  search_engines = vector < SingleAgentSearch* > (num_of_agents);
  agents_in_goal = vector <bool> (num_of_agents, false);

  for (int i = 0; i < num_of_agents; i++) {
    int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
    int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
    cout << "A" << i << endl;
    //cout << "Agent " << i << " - S: " << init_loc << " - G: " << goal_loc << endl;
    ComputeHeuristic ch(goal_loc, ml.get_map(), ml.rows*ml.cols, ml.actions_offset, e_w, &egr);
    search_engines[i] = new SingleAgentSearch(init_loc, goal_loc,
                                              ch.getHVals(),
                                              ml.get_map(), ml.rows*ml.cols, ml.actions_offset,
                                              &egr,
                                              e_w,
                                              tweak_g_val);
  }
  cout << ">> End Agents Initialization <<" << endl;
  cout << "--------->> Finds Paths (No constrains) <<---------" << endl;
  // initialize paths_found_initially (contain all individual optimal policies)
  bool all_agents_found_path = true;
  for (int i = 0; i < num_of_agents; i++){
    //cout << "---------------------------------------------------------------------" << endl;
    //cout << "---------- Agent " << i << " ----------" << endl;
    if ( search_engines[i]->findPath ( NULL, w_window_limit ) == false)
      all_agents_found_path = false;
  }
  if (all_agents_found_path == false) {
    cout << "\n>> NO SOLUTION EXISTS <<";
  } else {
    cout << "\n>> SOLUTION EXISTS <<" << endl;
    paths_found_initially.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
      paths_found_initially[i] = new vector<int> (*(search_engines[i]->getPath()));
  }

  paths = paths_found_initially;
  dummy_start = new CBSNode();
  dummy_start->agent_id = -1;
  dummy_start->g_val = compute_g_val(num_of_agents);
  dummy_start->open_handle = heap.push(dummy_start);
  
  //cout << "G_VAL " << dummy_start->g_val << endl;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CBSSearch::NextIteration(){
  heap.clear();
  solution_found = false;

  for (int agent = 0; agent < num_of_agents; agent++) {
    search_engines[agent]->start_location = paths[agent]->at(paths[agent]->size()-1);
  }

  bool all_agents_found_path = true;
  for (int i = 0; i < num_of_agents; i++){
    //cout << "---------------------------------------------------------------------" << endl;
    //cout << "---------- Agent " << i << " ----------" << endl;
    if ( search_engines[i]->findPath ( NULL, w_window_limit ) == false)
      all_agents_found_path = false;
  }
  if (all_agents_found_path == false) {
    cout << "\n>> NO SOLUTION EXISTS <<";
  } else {
    //cout << "\n>> SOLUTION EXISTS <<" << endl;
    paths_found_initially.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
      paths_found_initially[i] = new vector<int> (*(search_engines[i]->getPath()));
  }

  paths = paths_found_initially;
  dummy_start = new CBSNode();
  dummy_start->agent_id = -1;
  dummy_start->g_val = compute_g_val(num_of_agents);
  dummy_start->open_handle = heap.push(dummy_start);
}

bool CBSSearch::checkGoal() {
  bool done = false;
  for(int i=0; i<num_of_agents; i++){
    if(agents_in_goal[i] == false){
      done = false;
      break;      
    }
    else {done = true;}
  }
  return done;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CBSSearch::runCBSSearch() {
  int limit = int(w_window_limit/2)+1;
  // start is already in the heap
  
  //cout << "------------------------------------------>> Run CBS Search <<------------------------------------------" << endl << endl;
  //cout << "W_limit: " << w_window_limit << endl;

  std::clock_t start;
  double duration = 0;
  start = std::clock();
  
  while ( !heap.empty() && !solution_found ) {
    duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    if (duration > time_limit_cutoff || cbs_time > time_limit_cutoff) {  // timeout after 5 minutes
      cbs_time += duration;
      for(int i=0; i<num_of_agents; i++){
        this->low_expanded += search_engines[i]->num_expanded;
        this->low_generated += search_engines[i]->num_generated;
      }
      /*std::ofstream outfile;
      outfile.open("../Results/vanilla.csv", std::ios_base::app); //Empty results after 5 minutes limit
      string text = ";;;;;;"+std::to_string(duration)+";";
      outfile << text;
      outfile.close();*/
      solution_found = false;
      return false;
    }
    CBSNode* curr = heap.top();
    heap.pop();
    num_expanded++;
    curr->time_expanded = num_expanded;
    //    cout << "CBS NODE POPED FOR AG:" << curr->agent_id << endl;
    popped_nodes.push_front(curr);
    updatePaths(curr, dummy_start);  // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start

    /*cout << "---------------------------------------------------------------------------------------->>" << endl; 
    cout << "---------------------------------------------------------------------------------------->>" << endl; 
    cout << "CSB node agent " << curr->agent_id << endl;
    cout << "It has found the following paths as a child:" << endl;
    printPaths();
    cout << "---------------------------------------------------------------------------------------->>" << endl; 
    cout << "---------------------------------------------------------------------------------------->>" << endl;    */
   
    vector< tuple<int, int, int, int, int> >* collision_vec = extractCollisions(num_of_agents);  // check for collisions on updated paths
    /*    for (vector< tuple<int,int,int,int,int> >::const_iterator it = collision_vec->begin(); it != collision_vec->end(); it++)
      cout << "A1:" << get<0>(*it) << " ; A2:" << get<1>(*it) << " ; L1:" << get<2>(*it) << " ; L2:" << get<3>(*it) << " ; T:" << get<4>(*it) << endl;    */
    if ( collision_vec->size() == 0 ) {
      solution_found = true;
      //solution_cost += curr->g_val;
    } else {
      int agent1_id, agent2_id, location1, location2, timestep;
      tie(agent1_id, agent2_id, location1, location2, timestep) = collision_vec->at(0);
      CBSNode* n1 = new CBSNode();
      CBSNode* n2 = new CBSNode();
      n1->agent_id = agent1_id;
      n2->agent_id = agent2_id;
      if (location2 == -1) {  // generate vertex constraint
        n1->constraint = make_tuple(location1, -1, timestep);
        n2->constraint = make_tuple(location1, -1, timestep);
        //cout << "A" << n1->agent_id << " & A" << n2->agent_id << " in " << location1 << " timestep " << timestep << endl;
      } else {  // generate edge constraint
        n1->constraint = make_tuple(location1, location2, timestep);
        n2->constraint = make_tuple(location2, location1, timestep);
        //cout << "A" << n1->agent_id << " in " << location1 << " & " << location2 << " in timestep " << timestep << endl;
        //cout << "A" << n2->agent_id << " in " << location2 << " & " << location1 << " in timestep " << timestep << endl;
      }
      n1->parent = curr;
      n2->parent = curr;
      //cout << ">---------- Parent " << curr->agent_id << " ----------<" << endl;
      // find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them. Also updates n1's g_val
      /*cout << ">---------- Agent " << n1->agent_id << " ----------<" << endl;
      cout << "Path\t";
      int node = 0;
      for (vector<int>::const_iterator it = paths[n1->agent_id]->begin(); it != paths[n1->agent_id]->end(); ++it){
        std::cout << *it << '\t';
        //if(node == w_window_limit) break ; 
        node++;
      } cout << endl;*/

      if ( updateCBSNode(n1, dummy_start) == true ) {
        num_generated++;
        // new g_val equals old g_val plus the new path length found for the agent minus its old path length
        n1->g_val = curr->g_val - paths[n1->agent_id]->size() + n1->path.size();
        n1->open_handle = heap.push(n1);
      } else {
        delete (n1);
      }
      // same for n2
      /*cout << ">---------- Agent " << n2->agent_id << " ----------<" << endl;
      cout << "Path\t";
      node = 0;
      for (vector<int>::const_iterator it = paths[n2->agent_id]->begin(); it != paths[n2->agent_id]->end(); ++it){
        std::cout << *it << '\t';
        //if(node == w_window_limit) break ; 
        node++;
      } cout << endl;*/

      if ( updateCBSNode(n2, dummy_start) == true ) {
        num_generated++;
        n2->g_val = curr->g_val - paths[n2->agent_id]->size() + n2->path.size();
        n2->open_handle = heap.push(n2);
      } else {
        delete (n2);
      }
            /*cout << "---------------------------------------------------------------------------------------->>" << endl; 
            cout << "---------------------------------------------------------------------------------------->>" << endl; 

            cout << "It has found the following paths:" << endl;
            printPaths();
            //cout << "First node generated: " << *n1 << "Second node generated: " << *n2 << endl;
            cout << "---------------------------------------------------------------------------------------->>" << endl; 
            cout << "---------------------------------------------------------------------------------------->>" << endl; */
          
    }
    delete (collision_vec);
  }
  for(int i=0; i<num_of_agents; i++){
    //int last = paths[i]->at(paths[i]->size()-1);
    this->low_expanded += search_engines[i]->num_expanded;
    this->low_generated += search_engines[i]->num_generated;
    //this->open_updated += search_engines[i]->op_updated;
    //this->close_updated += search_engines[i]->cl_updated;
    
    //if(w_window_limit != -1 && (int) paths[i]->size() < w_window_limit+1) 
    //  paths[i]->resize(w_window_limit+1, last);
    if(w_window_limit != -1){
      if ((int) paths[i]->size() > limit) paths[i]->resize(limit); //Cut the path
      //if ((int) paths[i]->size() < limit) paths[i]->resize(limit, last); //Fill the path
    }

    //solution_cost += paths[i]->size();
    //paths_found_initially[*it]->resize(int(w_window_limit/2)+1); //Updates paths lenght to k movements (w/2)

    if(paths[i]->at(paths[i]->size()-1) == search_engines[i]->goal_location){
      agents_in_goal[i] = true;
    }
  }
  
  duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;

  cbs_time += duration;

  return solution_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CBSSearch::~CBSSearch() {
  for (size_t i = 0; i < search_engines.size(); i++)
    delete (search_engines[i]);
  for (size_t i = 0; i < paths_found_initially.size(); i++)
    delete (paths_found_initially[i]);
  //  for (size_t i=0; i<paths.size(); i++)
  //    delete (paths[i]);
  // clean heap memory and empty heap (if needed, that is if heap wasn't empty when solution found)
  for (heap_open_t::iterator it = heap.begin(); it != heap.end(); it++)
    delete (*it);
  heap.clear();
  // clean up other nodes (the ones that were popped out of the heap)
  //  cout << "Number of CBS nodes expanded: " << popped_nodes.size() << endl;
  while ( !popped_nodes.empty() ) {
    delete popped_nodes.front();
    popped_nodes.pop_front();
  }
}
