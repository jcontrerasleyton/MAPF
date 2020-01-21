#include "map_loader.h"
#include "agents_loader.h"
#include "egraph_reader.h"
#include "ecbs_search.h"
#include "cbs_search.h"
#include <string>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <utility>
#include <boost/program_options.hpp>

#include <ctime>

#ifndef EXTRA_PEN
#define EXTRA_PEN 0  /* Off by default. */
#endif


using namespace boost::program_options;
using namespace std;

int main(int argc, char** argv) {

  // Reading arguments ----------------------------------------------------------------
  cout << endl;
  string map_fname, agents_fname, name;
  // hwy_fname, search_method, results_fname;
  //double w_hwy, w_focal;
  int ins, agents;
  int time_limit;  // random restarts iterations number
  //bool tweakGVal, rand_succ_gen;
  try {
    options_description desc("Options");
    desc.add_options()
      ("help", "Print help messages")
      ("map", value<string>(&map_fname)->required(), "Map filename")
      //("agents", value<string>(&agents_fname)->required(), "Agents filename")
      //("highway", value<string>(&hwy_fname)->required(), "Highway filename or CRISSCROSS / GM / HONG")
      ("name", value<string>(&name)->required(), "name")
      ("agents", value<int>(&agents)->required(), "agents")
      ("instances", value<int>(&ins)->required(), "instances")
      //("search", value<string>(&search_method)->default_value("CBS"), "Search method (ECBS or iECBS. Default is ECBS)")
      //("tweakGVal", value<bool>(&tweakGVal)->default_value(false), "Change the cost structure or not (deprecated)")
      //("rand_succ_gen", value<bool>(&rand_succ_gen)->default_value(false), "Random order of successors generation (in the low-level search)")
      //("RRR", value<int>(&rrr_it)->default_value(0), "Random Restart #iterations (Default is 0, which runs once from root node for agentse ordered sequentially)")
      //("export_results", value<string>(&results_fname)->default_value("NONE"), "Results filename")
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

  MapLoader ml = MapLoader(map_fname);

  srand(time(NULL));
  int fin = 0;
  int fin2 = 0;
  int first = 0;
  int total_agents = 0;
  int instances = 0;
  //int area1 = 0;
  //int area2 = 0;
  bool option = true;

  fin = ml.rows-1; fin2 = ml.cols-1; first = agents; total_agents = agents+1; instances = ins; /*area1 = 6; area2 = 48;*/

  cout << "Map: " << map_fname << endl;

  //cout << "rows " << ml.rows << " cols " << ml.cols << endl;

  //int split_agents = (int)first / 2;
  //cout << split_agents << endl;

  for(int i = first; i < total_agents; i++){
    int total = 0;
    while(total < instances){
      int subtotal = 0;
      //int a1_sub = 0;
      //int a2_sub = 0;

      bool* aux_ini;
      bool* aux_goal;

      aux_ini = new bool[ml.rows*ml.cols];
      for (int j=0; j < ml.rows*ml.cols; j++)
        aux_ini[j] = ml.my_map[j];

      aux_goal = new bool[ml.rows*ml.cols];
      for (int j=0; j < ml.rows*ml.cols; j++)
        aux_goal[j] = ml.my_map[j];

      
      string path = "../../Instances/Input_old/"+name+"/Input_"+std::to_string(first);
      string command = "mkdir -p "+path;
      //cout << command << endl;
      const int dir_err = system(command.c_str());
      if (-1 == dir_err)
      {
        printf("Error creating directory!n");
        exit(1);
      }
      
      //string namefile = path+"/agent_"+std::to_string(i)+"_"+std::to_string(total)+".agents";
      string namefile = path+"/"+name+"_"+std::to_string(total)+".agents";
      std::ofstream outfile;
      outfile.open(namefile, std::ios_base::app);
      string text = std::to_string(i)+"\n";
      outfile << text;

      //cout << std::to_string(i) << endl;

      while(subtotal < i){

        int sx = (rand() % fin)+1;
        int sy = (rand() % fin2)+1;

        int gx = (rand() % fin)+1;
        int gy = (rand() % fin2)+1;

        int init_loc = ml.linearize_coordinate(sx,sy);
        int goal_loc = ml.linearize_coordinate(gx,gy);

        if(init_loc != goal_loc && aux_ini[init_loc] == false && aux_goal[goal_loc] == false){
          text = std::to_string(sx)+","+std::to_string(sy)+","+std::to_string(gx)+","+std::to_string(gy)+"\n";
          //cout << text;
          outfile << text;
          if(option){
            aux_ini[init_loc] = true;
            aux_goal[goal_loc] = true;
          }
          subtotal++;
        }
      }
      //cout << "--------------------------------------------" << endl;
      total++;
    }
  }

  cout << "All instances (" << first << " agents) generated" << endl;
}
