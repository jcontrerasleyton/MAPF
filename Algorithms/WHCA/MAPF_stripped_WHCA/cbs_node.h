// Represents 2D-Nodes
#ifndef CBSNODE_H
#define CBSNODE_H

#include <string>
#include <vector>
#include <list>
#include <climits>
#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>

using boost::heap::fibonacci_heap;
using boost::heap::compare;

using namespace std;

class CBSNode {
 public:
  int agent_id;
  tuple<int, int, int> constraint;  // <int loc1, int loc2, int timestep> NOTE--loc2=-1 for Vertex Constraint
  CBSNode* parent;
  vector<int> path;
  double g_val;  // (total cost)
  double h_val;
  int time_expanded;

  // the following is used to comapre nodes in the OPEN list
  struct compare_node {
    bool operator()(const CBSNode* n1, const CBSNode* n2) const {
      return n1->g_val > n2->g_val;
    }
  };  // used by OPEN to compare nodes by g-val (top of the heap has min g-val)

  typedef boost::heap::fibonacci_heap< CBSNode* , compare<compare_node> >::handle_type open_handle_t;

  open_handle_t open_handle;


  CBSNode();  // for efficiency, reserve num_of_agents in all_constraints vector
  static bool isEqual(const CBSNode* n1, const CBSNode* n2);
  ~CBSNode();


  // The following is used by googledensehash for checking whether two nodes are equal
  // we say that two nodes, s1 and s2, are equal if
  // both are non-NULL and have the same time_expanded (unique)
  struct cbs_eqnode {
    bool operator()(const CBSNode* s1, const CBSNode* s2) const {
      return (s1 == s2) || (s1 && s2 && s1->time_expanded == s2->time_expanded);
    }
  };

  // The following is used by googledensehash for generating the hash value of a nodes
  // this is needed because otherwise we'll have to define the specilized template inside std namespace
  struct CBSNodeHasher {
    std::size_t operator()(const CBSNode* n) const {
      size_t agent_id_hash = std::hash<int>()(n->agent_id);
      size_t time_expanded_hash = std::hash<int>()(n->time_expanded);
      return ( agent_id_hash ^ (time_expanded_hash << 1) );
    }
  };
};

std::ostream& operator<<(std::ostream& os, const CBSNode& n);

#endif
