#include "map_loader.h"
#include "agents_loader.h"
#include "egraph_reader.h"
#include "ecbs_search.h"
#include "cbs_search.h"
#include <string>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <utility>
#include <boost/program_options.hpp>
#include <experimental/filesystem>

#include <ctime>

#ifndef EXTRA_PEN
#define EXTRA_PEN 0  /* Off by default. */
#endif

namespace fs = std::experimental::filesystem;
using namespace boost::program_options;
using namespace std;

int main(int argc, char** argv) {
  cout << endl;
  // Reading arguments ----------------------------------------------------------------
//cout << "Hola" << endl;

  string map_fname, agents_fname, hwy_fname, search_method, results_fname;
  int w_window_limit; // Max number of timesteps per low_level search
  double w_hwy, w_focal;
  int rrr_it, time_limit;  // random restarts iterations number
  bool tweakGVal, rand_succ_gen;
  try {
    options_description desc("Options");
    desc.add_options()
      ("help", "Print help messages")
      ("map", value<string>(&map_fname)->required(), "Map filename")
      ("agents", value<string>(&agents_fname)->required(), "Agents filename")
      //("highway", value<string>(&hwy_fname)->required(), "Highway filename or CRISSCROSS / GM / HONG")
      ("focal_w", value<double>(&w_focal)->required(), "Focal weight")
      ("window_limit_w", value<int>(&w_window_limit)->required(), "Max number of timesteps per low_level search")
      ("highway_w", value<double>(&w_hwy)->required(), "Highway weight")
      ("search", value<string>(&search_method)->default_value("CBS"), "Search method (ECBS or iECBS. Default is ECBS)")
      ("tweakGVal", value<bool>(&tweakGVal)->default_value(false), "Change the cost structure or not (deprecated)")
      ("rand_succ_gen", value<bool>(&rand_succ_gen)->default_value(false), "Random order of successors generation (in the low-level search)")
      ("RRR", value<int>(&rrr_it)->default_value(0), "Random Restart #iterations (Default is 0, which runs once from root node for agentse ordered sequentially)")
      ("export_results", value<string>(&results_fname)->default_value("NONE"), "Results filename")
      ("time_limit",value<int>(&time_limit)->default_value(300), "Time limit cutoff [seconds]")
      ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
      cout << endl << desc << endl;
      return 0;
    }
    notify(vm);
  } catch(boost::program_options::required_option& e) {
    cout << endl << e.what() << endl;
    cout << "Use --help for arguments info." << endl << endl;
    return 0;
  }
  catch (exception& e) {
    cerr << endl << e.what() << endl;
    cout << "Use --help for arguments info." << endl << endl;
    return -1;
  }
  // ----------------------------------------------------------------------------------

//cout << std::boolalpha<< rand_succ_gen << endl;

  // read the map file and construct its two-dim array
  MapLoader ml = MapLoader(map_fname);

  // read agents' start and goal locations
  AgentsLoader al = AgentsLoader(agents_fname);

  // read the egraph (egraph file, experience_weight, weigthedastar_weight)
  EgraphReader egr;
/*
  if (hwy_fname.compare("CRISSCROSS") == 0) {
    egr = EgraphReader();
    egr.createCrissCrossHWY(&ml);
  } else if (hwy_fname.compare("GM") == 0) {
    LearnGMHWY* lgmhwy = new LearnGMHWY(map_fname, agents_fname);
    egr = *(lgmhwy->getHWY(1, w_hwy, 1));  // #iterations=1, w_hwy(=1 for first iteration), w_focal=1
  } else if (hwy_fname.compare("HONG") == 0) {
    HongHWY* honghwy = new HongHWY(map_fname, agents_fname);
    egr = *(honghwy->getHWY(100000,0.5,1.2,1.3));  // (#iterations, alpha, beta, gamma)
  } else {  // filename
*/
    egr = EgraphReader(hwy_fname);
//  }

  /*cout << search_method << " ; "
       << map_fname << " ; "
       << agents_fname << " ; "
       << hwy_fname << " ; "
       << w_hwy << " ; "
       << w_focal << " ; ";*/
    //       << std::boolalpha << tweakGVal << " ; ";
  //  cout << "PATH FOUND ; COST ; LB ; HL-EXP ; HL-GEN ; LL-EXP ; LL-GEN ; TIME[s]" << endl;
  //fflush(stdout);

  //ofstream res_f;
  //res_f.open(results_fname, ios::app);  // append the results file

  //bool res;
  //cout << rrr_it << " ; ";

	//ECBSSearch ecbs = ECBSSearch(ml, al, egr, w_hwy, w_focal, tweakGVal, rrr_it, rand_succ_gen);
  //CBSSearch::CBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, bool tweak_g_val) {
  //ECBSSearch::ECBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, double f_w, bool tweak_g_val, int rrr_it, bool rand_succ_gen) :

  //cout << "vanilla" << endl;
  cout << "limit: " << w_window_limit << endl << endl;

  string file = "../Results/whca_Input_"+std::to_string(al.num_of_agents)+"-w_"+std::to_string(w_window_limit)+".csv";

  std::ofstream outfile;
  outfile.open(file, std::ios_base::app);
  string name = fs::path(agents_fname).filename();
  string text = name+";"+std::to_string(w_window_limit)+";";
  //cout << text << endl;
  //string hola = "hola";
  outfile << text;
  outfile.close();

  //return 0;

  //outfile << text;
  
  clock_t start_s=clock();
  //cout << "\n\n>> CBS node creation <<" << endl;
  CBSSearch cbs = CBSSearch(ml, al, egr, w_hwy, tweakGVal, w_window_limit);
  
  cbs.time_limit_cutoff = time_limit;
  //cbs.w_window_limit = w_window_limit; 
  cout << "time limit: " << cbs.time_limit_cutoff << endl;
  //cout << ">> First iterarion paths <<" << endl;

	//ecbs.time_limit_cutoff = time_limit;
  //cbs.printPaths();

  cbs.runCBSSearch(); //Borrar

  //cout << ">> FINAL PATHS <<" << endl;
  //cbs.printPaths();

  bool done = false;
  int iteration = 0;

  if(!cbs.runCBSSearch()) {
    cbs.solution_found = false;
    done = true;
  }
  else done = cbs.checkGoal(); // Validate if every agent reach its goal

  if(w_window_limit == -1) done = true;

  //done = true;

  vector < vector<int>* > paths = cbs.paths;

  std::clock_t start;
  double duration = 0;
  start = std::clock();

  while(!done){
    duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    if (duration > cbs.time_limit_cutoff){
      cbs.solution_found = false;
      break;
    } 

    iteration++;

    //cout << "<<---------------------------------------------- Iteration " << iteration << "------------------------------------------------------>>" << endl;

    cbs.NextIteration();
    if(!cbs.runCBSSearch()) {cbs.solution_found = false; break;}

    //cout << ">> FINAL PATHS >> Iteration " << iteration << endl;
    //cbs.printPaths();

    for(int i=0; i<cbs.num_of_agents; i++) // Concatenate windowed paths
      paths[i]->insert(paths[i]->end(), cbs.paths[i]->begin()+1, cbs.paths[i]->end());

    done = cbs.checkGoal();
  }

  cbs.paths = paths;

  //cbs.printPaths();

  //cbs.solution_cost = cbs.solution_cost - (iteration*cbs.num_of_agents); // Get total cost (sum over the lenght of all paths)

  clock_t stop_s=clock();
  double total_time = (stop_s-start_s)/double(CLOCKS_PER_SEC);
  //cout << "time: " << total_time/**1000*/ << "s" << endl;

  //BORRAR
  //bool conflicts = false;
  //vector< tuple<int, int, int, int, int> >* collision_vec = cbs.extractCollisions(cbs.num_of_agents);
  //if(collision_vec->size() > 0) conflicts = true;

  int solution_cost_mov = 0;
  int makespand_mov = -1;
  int makespand = -1;

  if(cbs.solution_found){
    for(int i=0; i<cbs.num_of_agents; i++){

      int prev = -1;
      int goal = cbs.search_engines[i]->goal_location;
      int mov_cont = 0;
      int mov = (int) paths[i]->size();

      for (auto it = paths[i]->begin(); it != paths[i]->end(); ++it){ //Calcultes the solution's real cost
        if(*it != goal) mov_cont++;
        else if(*it != prev) mov_cont++;
        prev = *it;
      }
      mov_cont--;
      if(makespand_mov < mov_cont) makespand_mov = mov_cont;

      for (auto it = paths[i]->rbegin(); it != paths[i]->rend(); ++it){
        if(*it == goal) mov--;
        else break;
      }
      if(makespand < mov) makespand = mov; //Looks for the largest timestep

      solution_cost_mov += mov_cont;
      cbs.solution_cost += mov+1;
    }
  }

  
  if (cbs.solution_found) {
    cout << "FOUND COLLISION FREE PATH! Path cost:" << cbs.solution_cost;
    //cout << " ; High-level Expanded:" << cbs.num_expanded;
    //cout << " ; High-level Generated:" << cbs.num_generated;
    cout << " ; Low-level Expanded:" << cbs.low_expanded;
    cout << " ; Low-level Generated:" << cbs.low_generated << endl;

    std::ofstream outfile;
    outfile.open(file, std::ios_base::app);
    string text = "SUCCEED;";
    outfile << text;
    outfile.close();
 
    //    printPaths();
  } else {
    cout << "FAILED TO FIND A COLLISION FREE PATH :(" << " ; High-level Expanded:" << cbs.num_expanded << endl;

    std::ofstream outfile;
    outfile.open(file, std::ios_base::app);
    string text = "FAIL;";
    outfile << text;
    outfile.close();

  }

  //cout << "time: " << total_time/**1000*/ << "s" << endl;
  
  //std::ofstream outfile;
  outfile.open(file, std::ios_base::app);
  //string text = std::to_string(total_time)+"\n";
  text = std::to_string(cbs.num_of_agents)+";"+std::to_string(int(cbs.solution_cost));
  text += ";"+std::to_string(makespand)+";"+std::to_string(solution_cost_mov)+";"+std::to_string(makespand_mov);
  //text += ";"+std::to_string(cbs.num_expanded)+";"+std::to_string(cbs.num_generated);
  text += ";"+std::to_string(cbs.low_expanded)+";"+std::to_string(cbs.low_generated);
  text += ";"+std::to_string(iteration);
  text += ";"+std::to_string(cbs.cbs_time)+";"+std::to_string(total_time)+"\n";
  outfile << text;
  outfile.close();

  cbs.search_engines.clear();

  cout << "----------------------------------------------------------------------------------------------------" << endl;

}
