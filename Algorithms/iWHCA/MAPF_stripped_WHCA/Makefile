CXX = g++
CXXFLAGS = -Wall -DNDEBUG -I /usr/include/boost/ -I ../../../lib/ -std=c++14 -lboost_program_options -lboost_graph -O3
GUIFLAGS = -L/usr/local/lib64 -lfltk -lm -lX11 -lXext -lpthread -lXfixes -ldl -lglut -lGL -lfltk_gl -lboost_program_options -lboost_graph
GUROBIFLAGS = -L /home/lirchi/gurobi604/linux64/lib -lgurobi_c++ -lgurobi60 -I /home/lirchi/gurobi604/linux64/include
DAIFLAGS = -L /home/lirchi/Dropbox/libDAI/libDAI-0.3.2/lib -lgmpxx -lgmp -I /home/lirchi/Dropbox/libDAI/libDAI-0.3.2/include
DAILIB = /home/lirchi/Dropbox/libDAI/libDAI-0.3.2/lib/libdai.a
#TREEDFLAGS = -I /home/lsuazo/Other/CBS/MAPF_stripped_CBS_0print/TreeD/
TREEDFLAGS = -I TreeD/
#TREEDLIB = -L /home/lsuazo/Other/CBS/MAPF_stripped_CBS_0print/TreeD/ -ltreed
TREEDLIB = -L TreeD/ -ltreed

map_reader.o: map_loader.cpp map_loader.h
	$(CXX) $(CXXFLAGS) -c map_loader.cpp

agents_loader.o: agents_loader.cpp agents_loader.h
	$(CXX) $(CXXFLAGS) -c agents_loader.cpp

compute_heuristic.o: compute_heuristic.cpp compute_heuristic.h
	$(CXX) $(CXXFLAGS) -c compute_heuristic.cpp

node.o: node.cpp node.h
	$(CXX) $(CXXFLAGS) -c node.cpp

egraph_reader.o: egraph_reader.cpp egraph_reader.h
	$(CXX) $(CXXFLAGS) -c egraph_reader.cpp

single_agent_ecbs.o: single_agent_ecbs.cpp single_agent_ecbs.h
	$(CXX) $(CXXFLAGS) -c single_agent_ecbs.cpp

ecbs_node.o: ecbs_node.cpp ecbs_node.h
	$(CXX) $(CXXFLAGS) -c ecbs_node.cpp

graph_algo_wrapper.o: graph_algo_wrapper.cpp graph_algo_wrapper.h numvc.h
	$(CXX) $(CXXFLAGS) $(TREEDFLAGS) -c graph_algo_wrapper.cpp

ecbs_search.o: ecbs_search.cpp ecbs_search.h
	$(CXX) $(CXXFLAGS) -c ecbs_search.cpp

driver.o: driver.cpp
	$(CXX) $(CXXFLAGS) $(GUIFLAGS) -c driver.cpp

clean:
	rm -rf *o *~ driver driver.log core mapf_gui hwy_driver

backup:
	tar czvf backup.tar.gz *h *cpp *cxx Makefile

GridViewer.o: GridViewer.h GridViewer.cxx
	$(CXX) $(CXXFLAGS) $(GUIFLAGS) -c GridViewer.cxx

GUI_UI.o: GUI_UI.h GUI_UI.cxx
	$(CXX) $(CXXFLAGS) $(GUIFLAGS) -c GUI_UI.cxx

GUIMain.o: GUIMain.cxx
	$(CXX) $(CXXFLAGS) $(GUIFLAGS) -c GUIMain.cxx

gui: GUIMain.o GUI_UI.o GridViewer.o node.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o node.o egraph_reader.o single_agent_search.o cbs_node.o cbs_search.o direction_map.o single_agent_ecbs.o ecbs_node.o ecbs_search.o tansel_LL_search.o tansel_HL_search.o graph_algo_wrapper.o
	$(CXX) GridViewer.o GUI_UI.o GUIMain.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o single_agent_search.o node.o cbs_node.o cbs_search.o direction_map.o single_agent_ecbs.o ecbs_node.o ecbs_search.o tansel_LL_search.o tansel_HL_search.o graph_algo_wrapper.o $(CXXFLAGS) $(GUIFLAGS) $(TREEDFLAGS) $(TREEDLIB) -o mapf_gui

executable: driver.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o single_agent_search.o node.o cbs_node.o cbs_search.o single_agent_ecbs.o ecbs_node.o ecbs_search.o graph_algo_wrapper.o
	$(CXX) driver.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o single_agent_search.o node.o cbs_node.o cbs_search.o single_agent_ecbs.o ecbs_node.o ecbs_search.o graph_algo_wrapper.o $(CXXFLAGS) $(TREEDFLAGS) $(TREEDLIB) -o driver -lstdc++fs

profiler:
	echo "zzz" >> callgrind.out
	rm callgrind.out*
	valgrind --tool=callgrind ./driver corridor1.map corridor1.agents corridor1.hwy 2.0 1.1
	kcachegrind callgrind.out*

run_example:
	echo "./driver --map benchmark/maps/kiva_0.map --agents benchmark/agents_set2/kiva100_2.agents --highway benchmark/highways/kiva_0_1.hwy --focal_w=1.5 --highway_w=1 --search ECBS --RRR 20"
#e.g.: ./random_map_generator benchmark/maps/maze.map 5 1 15 10 2 1 15 6 20 15 1 20 6 15 1 20 6 1 15 6 20

make_tags:
	echo "find . -maxdepth 1 -regex '.*/.*\.\(c\|cpp\|h\|cxx\)$' -print | etags -"
