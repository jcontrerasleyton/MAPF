/************************************************
** This is a local search solver for Minimum Vertex Cover.                                                       
************************************************/


/************************************************
** Date:	2011.7.1  
** TSEWF (Two Stage Exchange and Weighting with Forgetting) 
** Author: Shaowei Cai, shaowei_cai@126.com    
**		   School of EECS, Peking University   
**		   Beijing, China                      
** 
** Date:    	2011.10.28
** Modify: Shaowei Cai
** use dynamic memory for v_adj[][] and v_edge[][], tidy codes.                                                        
************************************************/

/************************************************
** NuMVC Version 2011.11.7                                                    
************************************************/

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <utility>
#include <vector>

#include <sys/times.h>
#include <cmath>
#include <unistd.h>
#define STEP_DELAY 100

using namespace std;


#define pop_numvc(stack) stack[--stack ## _fill_pointer]
#define push_numvc(item, stack) stack[stack ## _fill_pointer++] = item

/*max vertex count and max edge count*/
#define	MAXV	10000
#define MAXE	50000000

int try_step=10000;

tms start, finish;
int start_time;

struct Edge{
	int v1;
	int v2;
};

/*parameters of algorithm*/
long long	max_steps;			//step limit
int			cutoff_time = 5;	//time limit
long long	step;
int			optimal_size = 0;	//terminate the algorithm before step limit if it finds a vertex cover of optimal_size

/*parameters of the instance*/
int		v_num;//|V|: 1...v
int		e_num;//|E|: 0...e-1

/*structures about edge*/
Edge	edge[MAXE];
int		edge_weight[MAXE];
double          vertex_weight[MAXV];

/*structures about vertex*/
double		dscore[MAXV];			//dscore of v
long long	time_stamp[MAXV];
int		best_cov_v;		//the vertex of the highest dscore in C

//from vertex to it's edges and neighbors
int*	v_edges[MAXV];	//edges related to v, v_edges[i][k] means vertex v_i's k_th edge
int*	v_adj[MAXV];		//v_adj[v_i][k] = v_j(actually, that is v_i's k_th neighbor)
int		v_edge_count[MAXV];	//amount of edges (neighbors) related to v


/* structures about solution */
//current candidate solution
int		c_size;						//cardinality of C
int		v_in_c[MAXV];				//a flag indicates whether a vertex is in C
int		remove_cand[MAXV];			//remove candidates, an array consists of only vertices in C, not including tabu_remove
int		index_in_remove_cand[MAXV];
int		remove_cand_size;

//best solution found
int		best_c_size;
int		best_v_in_c[MAXV];			//a flag indicates whether a vertex is in best solution
double  best_comp_time;
long    best_step;


//uncovered edge stack
int		uncov_stack[MAXE];		//store the uncov edge number
int		uncov_stack_fill_pointer;
int		index_in_uncov_stack[MAXE];//which position is an edge in the uncov_stack


//CC and taboo
int 	conf_change[MAXV];
int		tabu_remove=0;

//smooth 
int		ave_weight=1;
int		delta_total_weight=0;
int		threshold;
float	p_scale=0.3;//w=w*p


/* functions declaration */
int build_instance(char *filename);
void init_sol();
void cover_LS();
void add(int v);
void remove(int v);
void update_edge_weight();
void cover_rest_edges();
int check_solution();
int build_instance(int num_of_vertices, vector< pair<int, int> > edges);  // add by Liron ($$$)
int getVCSize(int seed, int optimal_size = 0, int time_limit = 5);  // add by Liron ($$$)


void update_best_sol()
{
	int i;

	for (i=1;i<=v_num;i++)
	{
		best_v_in_c[i] = v_in_c[i];
	}
	
	best_c_size = c_size;
	times(&finish);
	best_comp_time = double(finish.tms_utime - start.tms_utime + finish.tms_stime - start.tms_stime)/sysconf(_SC_CLK_TCK);
	best_comp_time = round(best_comp_time * 100)/100.0;
	best_step = step;
	
}



int build_instance(char *filename)
{
	char line[1024];
	char tempstr1[10];
	char tempstr2[10];
	int  v,e;
	
	char	tmp;
	int		v1,v2;
	
	ifstream infile(filename);
    if(infile.good()) return 0;

	/*** build problem data structures of the instance ***/
	infile.getline(line,1024);
    
	while (line[0] != 'p')
		infile.getline(line,1024);
	sscanf(line, "%s %s %d %d", tempstr1, tempstr2, &v_num, &e_num);

	/* read edges and compute v_edge_count */
	for (v=1; v<=v_num; v++) {
		v_edge_count[v] = 0;
		vertex_weight[v] = 1; // init w(v)=1
	}
	
	for (e=0; e<e_num; e++)
	{
		infile>>tmp>>v1>>v2;
		v_edge_count[v1]++;
		v_edge_count[v2]++;
		
		edge[e].v1 = v1;
		edge[e].v2 = v2;
	}

	// read vertices weights: each line 'w' v_num weight
	double v_weight;
	while (infile>>tmp>>v1>>v_weight) {
	  vertex_weight[v1]=v_weight;
	  //	  cout << tmp << " ; W(" << v1 << ")="<<v2 << endl;
	}
	infile.close();
	
	/* build v_adj and v_edges arrays */
	for (v=1; v<=v_num; v++)
	{
		v_adj[v] = new int[v_edge_count[v]];
		v_edges[v] = new int[v_edge_count[v]];
	}

	int v_edge_count_tmp[MAXV];	
	for(v=1; v<=v_num; v++)
		v_edge_count_tmp[v]=0;
	for (e=0; e<e_num; e++)
	{
		
		v1=edge[e].v1;
		v2=edge[e].v2;

		v_edges[v1][v_edge_count_tmp[v1]] = e;
		v_edges[v2][v_edge_count_tmp[v2]] = e;

		v_adj[v1][v_edge_count_tmp[v1]] = v2;
		v_adj[v2][v_edge_count_tmp[v2]] = v1;

		v_edge_count_tmp[v1]++;
		v_edge_count_tmp[v2]++;
	}

	return 1;

}


///////////////////////////////// Liron ///////////////////////////////////
int build_instance(int num_of_vertices, vector< pair<int, int> > edges) {
  int v,e;
  int v1,v2;
	
  v_num = num_of_vertices;
  e_num = edges.size();
  
  for (v = 1; v <= v_num; v++) {
    v_edge_count[v] = 0;
    vertex_weight[v] = 1;  // init w(v)=1
  }
	
  for (e = 0; e < e_num; e++) {
    v1 = edges[e].first;
    v2 = edges[e].second;
    v_edge_count[v1]++;
    v_edge_count[v2]++;	
    edge[e].v1 = v1;
    edge[e].v2 = v2;
  }
	
  /* build v_adj and v_edges arrays */
  for (v = 1; v <= v_num; v++) {
	v_adj[v] = new int[v_edge_count[v]];
	v_edges[v] = new int[v_edge_count[v]];
  }

  int v_edge_count_tmp[MAXV];	
  for(v = 1; v <= v_num; v++)
    v_edge_count_tmp[v] = 0;

  for (e = 0; e < e_num; e++) {
    v1 = edge[e].v1;
    v2 = edge[e].v2;
    v_edges[v1][v_edge_count_tmp[v1]] = e;
    v_edges[v2][v_edge_count_tmp[v2]] = e;
    v_adj[v1][v_edge_count_tmp[v1]] = v2;
    v_adj[v2][v_edge_count_tmp[v2]] = v1;
    v_edge_count_tmp[v1]++;
    v_edge_count_tmp[v2]++;
  }

  return 1;
}
///////////////////////////////////////////////////////////////////////////////


void free_memory()
{
	for (int v=1; v<=v_num; v++)
	{
		delete[] v_adj[v];
		delete[] v_edges[v];
	}
}

void reset_remove_cand()
{
	int v,j;
	j=0;
	for (v=1;v<=v_num;v++)
	{
		if(v_in_c[v]==1)// && v!=tabu_remove)
		{
			remove_cand[j] = v;
			index_in_remove_cand[v]=j;
			j++;
		}
		else index_in_remove_cand[v]=0;
	}
	
	remove_cand_size = j;
	
}




void update_target_size()
{

  /*  
  cout << "update_target_size()  cover_size=" << c_size << endl;
  for (int v=1; v<=v_num; ++v)
    if (v_in_c[v]==1)
      cout << "W(V_" << v <<")=" << vertex_weight[v] << " with dscore="<< dscore[v] <<",";
  cout << endl;
  */

	//	int v;
	int max_improvement;
	int max_vertex=-1;//vertex with the highest improvement in C

	max_improvement=-100000000;
	for (int v=1; v<=v_num; ++v)
	{
		if(v_in_c[v]==0)continue;
		if (dscore[v]>max_improvement)
		{
			max_improvement = dscore[v];
			max_vertex = v;
		}
	}
	if (max_improvement > -0.5) {
	  remove(max_vertex);
	  reset_remove_cand();
	  c_size--;		 
	}
}




//update the best vertex in C 

void update_best_cov_v()
{
	int i,v;
	best_cov_v = remove_cand[0];
	for (i=1; i<remove_cand_size; ++i)
	{
		v = remove_cand[i];
		if(v==tabu_remove) continue;
		
		if( dscore[v] < dscore[best_cov_v])
			continue;
		else if( dscore[v]> dscore[best_cov_v] )
			best_cov_v = v;
		else if (time_stamp[v]<time_stamp[best_cov_v])
			best_cov_v = v;
	}
}



inline
void uncover(int e) 
{
	index_in_uncov_stack[e] = uncov_stack_fill_pointer;
	push_numvc(e,uncov_stack);
}


inline
void cover(int e)
{
	int index,last_uncov_edge;

	//since the edge is satisfied, its position can be reused to store the last_uncov_edge
	last_uncov_edge = pop_numvc(uncov_stack);
	index = index_in_uncov_stack[e];
	uncov_stack[index] = last_uncov_edge;
	index_in_uncov_stack[last_uncov_edge] = index;
}

void init_sol()
{
	int i,v,e;

	/*** build solution data structures of the instance ***/
	//init vertex cover
	for (v=1; v<=v_num; v++)
	{
		v_in_c[v] = 0;
		dscore[v] = 0;
		
		conf_change[v] = 1;
		time_stamp[v]= 0; // to break ties
	}

	for (e=0; e<e_num; e++)
	{
		edge_weight[e] = 1;
		dscore[edge[e].v1]+=edge_weight[e]/vertex_weight[edge[e].v1];
		dscore[edge[e].v2]+=edge_weight[e]/vertex_weight[edge[e].v2];
	}

	//init uncovered edge stack and cover_vertrex_count_of_edge array
	uncov_stack_fill_pointer = 0;
	for (e=0; e<e_num; e++)
		uncover(e);

	int best_vertex_improvement;
	int best_count;
	int best_array[MAXV];

	for (i=0; uncov_stack_fill_pointer>0; )
	{
		best_vertex_improvement = 0;
		best_count = 0;
		for (v=1; v<=v_num; ++v)
		{
			if(v_in_c[v]==1)continue;

			if (dscore[v]>best_vertex_improvement)
			{
				best_vertex_improvement = dscore[v];
				best_array[0] = v;
				best_count = 1;
			}
			else if (dscore[v]==best_vertex_improvement)
			{
				best_array[best_count] = v;
				best_count++;
			}
		}

		if(best_count>0)
		{
			add(best_array[rand()%best_count]);
			++i;
		}
	}

	c_size = i;
	
	update_best_sol();
	
	reset_remove_cand();
	
	update_best_cov_v();
	
}


void add(int v)
{
	v_in_c[v] = 1;
	dscore[v] = -dscore[v];
	
	int i,e,n;

	int edge_count = v_edge_count[v];
	
	for (i=0; i<edge_count; ++i)
	{
		e = v_edges[v][i];// v's i'th edge
		n = v_adj[v][i];//v's i'th neighbor

		if (v_in_c[n]==0)//this adj isn't in cover set
		{
			dscore[n] -= edge_weight[e]/vertex_weight[n];
			conf_change[n] = 1;

			cover(e);
		}
		else
		{
			dscore[n] += edge_weight[e]/vertex_weight[n];
		}
	}
	
}

void remove(int v)
{
	v_in_c[v] = 0;
	dscore[v] = -dscore[v];
	conf_change[v] = 0;

	int i,e,n;

	int edge_count = v_edge_count[v];
	for (i=0; i<edge_count; ++i)
	{
		e = v_edges[v][i];
		n = v_adj[v][i];

		if (v_in_c[n]==0)//this adj isn't in cover set
		{
			dscore[n] += edge_weight[e]/vertex_weight[n];
			conf_change[n] = 1;

			uncover(e);
		}
		else
		{
			dscore[n] -= edge_weight[e]/vertex_weight[n];
		}
	}

}

/*On solution*/

void print_solution()
{
	int mis_vertex_count=0;
	
	for (int i=1; i<=v_num; i++)
	{
		if (best_v_in_c[i]!=1)
			mis_vertex_count++;
	}
	
	if(mis_vertex_count+best_c_size!=v_num)
		cout<<"The size of independent set + the size of vertex cover is not equal to |V(G)|!"<<endl;
	
	cout<<"c Best found independent set size = "<<mis_vertex_count<<endl;
	cout<<"c The following output is the found independent set."<<endl;


	for (int i=1; i<=v_num; i++)
	{
		if (best_v_in_c[i]!=1)//output max independent set
			cout<<i<<'\t';
	}
	cout<<endl;

}

//check whether the solution found is a proper solution
int check_solution()
{
	int e;
	
	for(e=0; e<e_num; ++e)
	{
		if(best_v_in_c[edge[e].v1]!=1 && best_v_in_c[edge[e].v2]!=1)
		{
			cout<<"uncovered edge "<<e<<endl;
			return 0;
		}
	}
	
	return 1;
}

void forget_edge_weights() {
	int v,e;
	int new_total_weight=0;
	
	for(v=1; v<=v_num; v++)
		dscore[v]=0;

	//scale_ave=ave_weight*q_scale;
	for (e = 0; e<e_num; e++)
	{
		edge_weight[e] = edge_weight[e]*p_scale;
			
		new_total_weight+=edge_weight[e];
		
		//update dscore
		if (v_in_c[edge[e].v1]+v_in_c[edge[e].v2]==0){
			dscore[edge[e].v1]+=edge_weight[e]/vertex_weight[edge[e].v1];
			dscore[edge[e].v2]+=edge_weight[e]/vertex_weight[edge[e].v2];
			}
		else if(v_in_c[edge[e].v1]+v_in_c[edge[e].v2]==1){
			if(v_in_c[edge[e].v1]==1)
			  dscore[edge[e].v1]-=edge_weight[e]/vertex_weight[edge[e].v1];
			else
			  dscore[edge[e].v2]-=edge_weight[e]/vertex_weight[edge[e].v2];
		}
	}
	ave_weight=new_total_weight/e_num;

}


void update_edge_weight()
{
	int i,e;
	for(i=0; i<uncov_stack_fill_pointer; ++i)
	{
		e = uncov_stack[i];
		
		edge_weight[e]+= 1;
		dscore[edge[e].v1] += 1/vertex_weight[edge[e].v1];
		dscore[edge[e].v2] += 1/vertex_weight[edge[e].v2];
	}
	
	
	delta_total_weight+=uncov_stack_fill_pointer;
	if(delta_total_weight>=e_num)
	{
		ave_weight+=1;
		delta_total_weight -= e_num;
	}
	
	//smooth weights
	if(ave_weight>=threshold)
	{
		forget_edge_weights();
	}
	
}


void cover_LS()
{
	int		best_add_v;
	int		e,v1,v2;
    //	int step_marker;
	//	int		i;

	step = 1;
    //	step_marker = -100;

	while(1)// wihin cutoff_time
	//while(step<=max_steps)
	{
	  if ( uncov_stack_fill_pointer == 0 )//update best solution if needed
		{
          //		  cout << "PART A:" << endl;
          //		  print_solution();
		  update_best_sol();
		  //		  if (c_size==optimal_size)
		  //		    return;
		  
		  update_target_size();
          //	  step_marker = step;

			times(&finish);
			
			double elap_time = (finish.tms_utime + finish.tms_stime - start_time)/sysconf(_SC_CLK_TCK);
	
			if(elap_time >= cutoff_time)return;


			//			continue;
		}
		
		//if(step>=try_step)// wihin cutoff_time
		if(step%try_step==0)
		{
          //		  cout << "PART B:" << endl;
          //		  print_solution();
		
			times(&finish);
			
			double elap_time2 = (finish.tms_utime + finish.tms_stime - start_time)/sysconf(_SC_CLK_TCK);
	
			if(elap_time2 >= cutoff_time)return;
			 
			 //double steps_per_sec = (double)try_step/elap_time;
			 
			 //try_step += (int)(cutoff_time-elap_time)*steps_per_sec;
		}
		
	
		update_best_cov_v();

		remove(best_cov_v);

        if (uncov_stack_fill_pointer < 1)  // $$$ -- Liron (fixed divide by zero exception)
          return;
		e = uncov_stack[rand()%uncov_stack_fill_pointer];

		v1 = edge[e].v1;
		v2 = edge[e].v2;
		
		if(conf_change[v1]==0 ) best_add_v=v2;
		else if(conf_change[v2]==0) best_add_v=v1;
		
		else{
			if(dscore[v1]>dscore[v2] || (dscore[v1]==dscore[v2] && time_stamp[v1]<time_stamp[v2]) )
				best_add_v=v1;
			else best_add_v=v2;
		}

		add(best_add_v);
		
		int index = index_in_remove_cand[best_cov_v];
		index_in_remove_cand[best_cov_v] = 0;
		
		remove_cand[index] = best_add_v;
		index_in_remove_cand[best_add_v] = index;
		
		
		//update remove_cand
		/*int index = index_in_remove_cand[best_cov_v];
		index_in_remove_cand[best_cov_v] = 0;
		
		
		if(v_in_c[tabu_remove]==1 && index_in_remove_cand[tabu_remove]==0)
		{
			remove_cand[index] = tabu_remove;
			index_in_remove_cand[tabu_remove] = index;
		}
		else
		{
			int v;
			
			for(i=index; i<remove_cand_size-1; ++i)
			{	
				v = remove_cand[i+1];
				remove_cand[i] = v;
				index_in_remove_cand[v] = i;	
			}
			remove_cand_size--;
		}*/
		
		time_stamp[best_add_v]=time_stamp[best_cov_v]=step;

		tabu_remove = best_add_v;
		
		update_edge_weight();
		
		step++;
	  }
}

/////////////////////////////  Liron  ///////////////////////////////////////
// note -- assumes graph already been loaded...
int getVCSize(int seed, int optimal_s, int time_limit) {
  optimal_size = optimal_s;
  cutoff_time = time_limit;
  
  threshold = (int)(0.5*v_num); 
	
  srand(seed);

  times(&start);
  start_time = start.tms_utime + start.tms_stime;

  init_sol();
    
  if(c_size + uncov_stack_fill_pointer > optimal_size ) {
    cover_LS();
  }
		
  //check solution
  if(check_solution()==1) {
    return best_c_size;
  }	else {
    return -1;
  }
  
  free_memory();
  return -1;
}
