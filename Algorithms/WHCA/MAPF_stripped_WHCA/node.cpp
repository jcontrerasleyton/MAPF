#include "node.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

using namespace boost;
using namespace std;

Node::Node() : id(0), g_val(0), h_val(0), parent(NULL), timestep(0), num_internal_conf(0), in_openlist(false),
	       g_hwy_val(0), h_hwy_val(0) {
}

Node::Node(int id, double g_val, double h_val, Node* parent, int timestep, int num_internal_conf, bool in_openlist, double g_hwy_val, double h_hwy_val):
    id(id),  g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
    num_internal_conf(num_internal_conf), in_openlist(in_openlist), g_hwy_val(g_hwy_val), h_hwy_val(h_hwy_val) {
}

Node::Node(const Node& other) {
  id = other.id;
  g_val = other.g_val;
  h_val = other.h_val;
  parent = other.parent;
  timestep = other.timestep;
  in_openlist = other.in_openlist;
  open_handle = other.open_handle;
  focal_handle = other.focal_handle;
  num_internal_conf = other.num_internal_conf;
  g_hwy_val = other.g_hwy_val;
  h_hwy_val = other.h_hwy_val;
}

Node::~Node() {
}

std::ostream& operator<<(std::ostream& os, const Node& n) {
  if ( n.parent != NULL )
    os << "ID=" << n.id << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << std::setprecision(4) << n.h_val
       << " ; #CONF="<< n.num_internal_conf << " ; PARENT=" << (n.parent)->id
       << " ; IN_OPEN?" << std::boolalpha << n.in_openlist;
  else
    os << "ID=" << n.id << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << std::setprecision(4) << n.h_val
       << " ; #CONF="<< n.num_internal_conf << " ; ROOT (NO PARENT)";
  return os;
}
/*std::ostream& operator<<(std::ostream& os, const Node* n) {
  os << "ID=" << n->id << " ; TIMESTEP=" << n->timestep << " ; GVAL=" << n->g_val << " ; PARENT=" << (n->parent)->id;
  return os;
  }*/
