#include <boost/heap/fibonacci_heap.hpp>
#include "compute_heuristic.h"
#include <cstring>
#include <climits>
#include <cfloat>
#include "../../include/sparsehash/dense_hash_map"
#include "node.h"

using google::dense_hash_map;      // namespace where class lives by default
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;

/*
// this is needed because otherwise we'll have to define the specilized template inside std namespace
struct NodeHasher
{
  std::size_t operator()(const Node* n) const
  {
    using std::tr1::hash;
    //    cout << "COMPUTE HASH: " << *n << " ; Hash=" << hash<int>()(n->id) << endl;
    //cout << "   Pointer Address: " << n << endl;
   return ( hash<int>()(n->id) );
  }
};
*/


ComputeHeuristic::ComputeHeuristic(int goal_location, const bool* my_map, int map_size,
                                   const int* actions_offset, double e_weight, const EgraphReader* egr) :
    my_map (my_map), actions_offset(actions_offset) {
  this->egr = egr;
  this->e_weight = e_weight;
  this->goal_location = goal_location;
  this->map_size = map_size;
  h_vals = new double[map_size];
  for (int i = 0; i < map_size; i++)
    h_vals[i] = DBL_MAX;
  // generate a heap that can save nodes (and a open_handle)
  boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::compare_node> > heap;
  boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::compare_node> >::handle_type open_handle;
  // generate hash_map (key is a node pointer, data is a node handler,
  //                    NodeHasher is the hash function to be used,
  //                    eqnode is used to break ties when hash values are equal)
  dense_hash_map<Node*, fibonacci_heap<Node* , boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode> nodes;
  nodes.set_empty_key(NULL);
  dense_hash_map<Node*, fibonacci_heap<Node* , boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode>::iterator it;

  Node* goal = new Node (goal_location, 0, 0, NULL, 0, false); //(id, g, h, parent, timestep, internal conflic)
  goal->open_handle = heap.push(goal);  // add goal to heap
  nodes[goal] = goal->open_handle;       // add goal to hash_table (nodes)
  while ( !heap.empty() ) {
    Node* curr = heap.top(); heap.pop();
    // cout << endl << "CURRENT node: " << curr << endl;
    for (int direction = 0; direction < 5; direction++) {
      int next_id = curr->id + actions_offset[direction];
      // cout << "next id=" << next_id << endl;
      if ( !my_map[next_id] ) {  // if that grid is not blocked
        // cout << "it is NOT blocked" << endl;
        // compute cost to next_id via curr node
        double cost = 1;
        /* "normal" EG */
        if ( ! (*egr).isEdge (next_id, curr->id) ) {
          cost = cost * e_weight;
          //cout << "dddf" << endl;
        } // check if the (inverse) directed edge (back from goal) is in the e_graph
        /* This is good for TCBS and bad for ECBS+HWY (because we inflate further)
        if ( (*egr).isEdge (curr->id, next_id) )  // check if we're going against the edge
          cost = cost * 2;
        */
        /* Check Tansel's suggestion for penalizing just going against highway
        if ( (*egr).isEdge (next_id, curr->id) )
          cost = cost * e_weight;
        */
        double next_g_val = curr->g_val + cost;
        // generate (maybe temporary) node
        Node* next = new Node (next_id, next_g_val, 0, NULL, 0, false);  // no timestep, h_val or parent here...
        // try to retrieve it from the hash table
        it = nodes.find(next);
        // cout << "For that node COUNT="<< nodes.count(&next) << endl;
        if ( it == nodes.end() ) {  // add the newly generated node to heap and hash table
          // cout << "ADDING new node: " << next << endl;
          next->open_handle = heap.push(next);
          nodes[next] = next->open_handle;
        } else {  // update existing node's g_val if needed (only in the heap)
          delete(next);  // not needed anymore -- we already generated it before
          Node* existing_next = (*it).first;
          open_handle = (*it).second;  // TODO: check if (*open_handle).g_val points to the same places as existing_next->g_val
          // cout << "Check if UPDATE needed for existing node: " << *existing_next << endl;
          if (existing_next->g_val > next_g_val) {
            existing_next->g_val = next_g_val;
            heap.update(open_handle);
             //cout << "   YES! Updated values are:" << *existing_next << endl;
          }
        }
      }
    }
  }
  // iterate over all nodes and populate the h_vals
  for (it=nodes.begin(); it != nodes.end(); it++) {
    Node* s = (*it).first;
    h_vals [s->id] = s->g_val;
    delete (s);
  }
  nodes.clear(); // maybe not needed
  heap.clear();  // maybe not needed
}

double* ComputeHeuristic::getHVals() {
  double* retVal = new double [ this->map_size ];
  memcpy (retVal, this->h_vals, sizeof(double) * this->map_size );
  return retVal;
}

ComputeHeuristic::~ComputeHeuristic() {
  delete[] this->h_vals;
  delete[] my_map;
}
