#include "cbs_node.h"
#include <vector>


CBSNode::CBSNode() {
  agent_id = -1;
  constraint = make_tuple(-1, -1, -1);
  path = vector<int>();
  g_val = 0;
  h_val = 0;
  time_expanded = -1;
}


bool CBSNode::isEqual(const CBSNode* n1, const CBSNode* n2) {
  return (n1->parent == n2->parent &&
          n1->agent_id == n2->agent_id &&
          n1->constraint == n2->constraint);
}


CBSNode::~CBSNode() {
}


std::ostream& operator<<(std::ostream& os, const CBSNode& n) {
  os << "THIS NODE HAS: g_val=" << n.g_val << " and h_val=" << n.h_val << ". It constrains agent " << n.agent_id <<
      " on loc1[" << get<0>(n.constraint) << "] loc2[" << get<1>(n.constraint) << "] at time[" << get<2>(n.constraint) << "]" << endl;
  return os;
}
