#include "single_agent_search.h"
#include <cstring>
#include <climits>
#include <vector>
#include <list>
#include <utility>

#include <boost/heap/fibonacci_heap.hpp>
#include "../../include/sparsehash/dense_hash_map"

#include "node.h"

using google::dense_hash_map;      // namespace where class lives by default
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;


SingleAgentSearch::SingleAgentSearch(int start_location, int goal_location, const double* my_heuristic,
                                     const bool* my_map, int map_size, const int* actions_offset,
                                     const EgraphReader* egr, double e_weight, bool tweak_g_val) :
  my_heuristic(my_heuristic), my_map(my_map), actions_offset(actions_offset), egr(egr) {
  this->start_location = start_location;
  this->goal_location = goal_location;
  this->map_size = map_size;
  this->e_weight = e_weight;
  this->tweak_g_val = tweak_g_val;
  this->num_expanded = 0;
  this->num_generated = 0;
  this->numf_generated = 0;
  this->path_cost = 0;
  // initialize allNodes_table (hash table)
  empty_node = new Node();
  empty_node->id = -1;
  deleted_node = new Node();
  deleted_node->id = -2;
  allNodes_table.set_empty_key(empty_node);
  allNodes_table.set_deleted_key(deleted_node);
}


void SingleAgentSearch::updatePath(Node* goal) {
  path.clear();
  Node* curr = goal;
  // cout << "   UPDATING Path for one agent to: ";
  while (curr->timestep != 0) {
    path.push_back(curr->id);
    // cout << curr->id << ",";
    // cout << curr->id << "(" << curr->timestep << "), ";
    curr = curr->parent;
  }
  path.push_back(start_location);
  // cout << start_location << endl;
  // cout << start_location << "(" << curr->timestep << ")" << endl;
  reverse(path.begin(), path.end());

  /*cout << "\t";
  for(size_t i = 0; i < path.size(); i++){
    cout << i << "\t";
  } cout << endl;

  cout << "Path:\t";
  for(size_t i = 0; i < path.size(); i++){
    cout << path[i] << "\t";
  } cout << endl;*/

  path_cost = goal->g_val;
}

inline void SingleAgentSearch::releaseClosedListNodes(hashtable_t* allNodes_table) {
  hashtable_t::iterator it;
  for (it=allNodes_table->begin(); it != allNodes_table->end(); it++) {
    delete ( (*it).second );  // Node* s = (*it).first; delete (s); (note -- it.first is the key and it.second is value)
  }
}


// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
int SingleAgentSearch::extractLastGoalTimestep(int goal_location, const vector< list< pair<int, int> > >* cons) {
  if (cons != NULL) {
    for ( int t = static_cast<int>(cons->size())-1; t > 0; t-- ) {
      for (list< pair<int, int> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it) {
        // $$$: in the following if, do we need to check second (maybe cannot happen in edge constraints?)
        if ((*it).first == goal_location || (*it).second == goal_location) {
          return (t);
        }
      }
    }
  }
  return -1;
}

/*
  Deprecated -- extractLastGoalTimestep used instead and is more efficient.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Return true if there are constraints that involve the goal_location after the curr_timestep
inline bool SingleAgentSearch::checkFutureConstraints(int goal_location, int curr_timestep, const vector< list< pair<int, int> > >* cons) {
  // cout << "CURR_T:" << curr_timestep << " ; CONS_SIZE:" << cons->size() << endl; fflush(stdout);
  if (cons != NULL) {
    if ( curr_timestep < static_cast<int>(cons->size()) ) {
      for (int t = curr_timestep; t < static_cast<int>(cons->size()); t++) {
        for (list< pair<int, int> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it) {
          // $$$: in the following if, do we need to check second (maybe cannot happen in edge constraints?)
          if ((*it).first == goal_location || (*it).second == goal_location) {
            return true;
          }
        }
      }
    }
  }
  return false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
//        cons[timestep] is a list of <loc1,loc2> of (vertex/edge) constraints for that timestep.
inline bool SingleAgentSearch::isConstrained(int curr_id, int next_id, int next_timestep, const vector< list< pair<int, int> > >* cons) {
  //  cout << "check if ID="<<id<<" is occupied at TIMESTEP="<<timestep<<endl;
  if (cons == NULL)
    return false;

  // check vertex constraints (being in next_id at next_timestep is disallowed)
  if ( next_timestep < static_cast<int>(cons->size()) ) {
    for ( list< pair<int, int> >::const_iterator it = cons->at(next_timestep).begin(); it != cons->at(next_timestep).end(); ++it ) {
      if ( (*it).second == -1 ) {
        if ( (*it).first == next_id ) {
          return true;
        }
      }
    }
  }

  // check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
  if ( next_timestep > 0 && next_timestep - 1 < static_cast<int>(cons->size()) ) {
    for ( list< pair<int, int> >::const_iterator it = cons->at(next_timestep-1).begin(); it != cons->at(next_timestep-1).end(); ++it ) {
      if ( (*it).first == curr_id && (*it).second == next_id ) {
        return true;
      }
    }
  }

  return false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
bool SingleAgentSearch::findPath(const vector < list< pair<int, int> > >* constraints, vector<int> left, int timestep, int type) {
  // clear data structures if they had been used before
  // (note -- nodes are deleted before findPath returns)
  open_list.clear();
  focal_list.clear();
  allNodes_table.clear();

  hashtable_t::iterator it;  // will be used for find()

  // generate start and add it to the OPEN list
  //(id, g, h, parent, timestep)
  Node* start = new Node(start_location, 0, my_heuristic[start_location], NULL, 0);
  this->num_generated++;
  start->open_handle = open_list.push(start);
  start->in_openlist = true;
  allNodes_table[start] = start;

  if(type == 0 && timestep > 1){
    Node* curr = start;
    open_list.pop();
    curr->in_openlist = false;
    for(int i = 1; i < timestep; i++){
      int next_id = left[i];
      int next_timestep = curr->timestep + 1;
      double next_g_val = curr->g_val + 1;
      double next_h_val = my_heuristic[next_id];
      Node* next = new Node (next_id, next_g_val, next_h_val, curr, next_timestep);
      //num_generated++;
      //num_expanded++;
      numf_generated++;
      allNodes_table[next] = next;
      curr = next;
    }
    curr->open_handle = open_list.push(curr);
    curr->in_openlist = true;
  }

  int lastGoalConsTime = extractLastGoalTimestep(goal_location, constraints);

  while ( !open_list.empty() ) {
    // pop OPEN head
    Node* curr = open_list.top(); open_list.pop();
    curr->in_openlist = false;
    num_expanded++;
    //cout << "ID: " << curr->id << " - Timestep " << curr->timestep << endl;

    if(type == 0 && curr->timestep > timestep && timestep < (int)left.size()-1){
      for(int i = timestep+1; i < (int)left.size()-1; i++){
        if(curr->id == left[i] && curr->timestep > lastGoalConsTime){
          //cout << ">> Found adaptive GOAL <<" << endl;
          for(int j = i+1; j < (int)left.size(); j++){
            int next_id = left[j];
            int next_timestep = curr->timestep + 1;
            double next_g_val = curr->g_val + 1;
            double next_h_val = my_heuristic[next_id];
            Node* next = new Node (next_id, next_g_val, next_h_val, curr, next_timestep);
            curr = next;
            //num_generated++;
            //num_expanded++;
            numf_generated++;
          }
          updatePath(curr);
          releaseClosedListNodes(&allNodes_table);
          return true;
        }
      }
    }

    if (curr->id == goal_location && curr->timestep > lastGoalConsTime) {
      //cout << "-Path found!-" << endl;
      updatePath(curr);
      releaseClosedListNodes(&allNodes_table);
      return true;
    }
    // If current node is not goal, generate successors
    for (int direction = 0; direction < 5; direction++) {  // {North,East,South,West,NoOP}
      int next_id = curr->id + actions_offset[direction];
      int next_timestep = curr->timestep + 1;
      if ( !my_map[next_id] && !isConstrained(curr->id, next_id, next_timestep, constraints) ) {  // if that grid is not blocked
        // compute cost to next_id via curr node
        double cost = 1;
        if (tweak_g_val == true)
          if ( !(this->egr)->isEdge(curr->id, next_id) ) {
            cost = cost * e_weight;
            // cout << next_id << "->" << curr->id << "  inflated with cost=" << cost << endl;
          }
        double next_g_val = curr->g_val + cost;
        double next_h_val = my_heuristic[next_id];
        // generate (maybe temporary) node
        Node* next = new Node (next_id, next_g_val, next_h_val, curr, next_timestep);
        // cout << "   NEXT(node)=" << next << endl;
        // try to retrieve it from the hash table
        it = allNodes_table.find(next);
        if ( it == allNodes_table.end() ) {  // add the newly generated node to open_list and hash table
          //  cout << "ADDING new node: " << next << endl;
          next->open_handle = open_list.push(next);
          next->in_openlist = true;
          num_generated++;
          allNodes_table[next] = next;
          // cout << "NEXT=" << next->id << " ; TIMESTEP=" << next->timestep << " ; g-val=" << next->g_val << " ; h_val=" << next->h_val <<  endl;
        } else {  // update existing node's if needed (only in the open_list)
          delete(next);  // not needed anymore -- we already generated it before
          Node* existing_next = (*it).second;
          if (existing_next->in_openlist == true) {
            if ( existing_next->g_val + existing_next->h_val > next_g_val + next_h_val ) {
              existing_next->g_val = next_g_val;
              existing_next->h_val = next_h_val;
              existing_next->parent = curr;
              // existing_next->timestep = next_timestep;
              open_list.increase(existing_next->open_handle);  // open_list.update(open_handle);
            }
          } else {  // if its in the closed list
            if ( existing_next->g_val + existing_next->h_val > next_g_val + next_h_val ) {
              existing_next->g_val = next_g_val;
              existing_next->h_val = next_h_val;
              existing_next->parent = curr;
              // existing_next->timestep = next_timestep;
              existing_next->open_handle = open_list.push(existing_next);
              existing_next->in_openlist = true;
              // allNodes_table[existing_next] = existing_next;
            }
          }
        }
      }
    }
  }
  // no path found
  //cout << "No Path found" << endl;
  path.clear();
  releaseClosedListNodes(&allNodes_table);
  return false;
}

SingleAgentSearch::~SingleAgentSearch() {
  //  delete[] this->my_heuristic; (created once and should be deleted from outside)
  delete[] my_map;
  delete[] my_heuristic;
  delete (empty_node);
  delete (deleted_node);
}
