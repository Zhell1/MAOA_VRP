#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include <map>

#include <lemon/list_graph.h>
#include <lemon/concepts/maps.h>
#include <lemon/dijkstra.h>
#include <lemon/nagamochi_ibaraki.h>

using namespace std;

/****************************  C_edge  *******************************/
class C_link{
public:
  int num;      // Number of the edge

  //thomas: v1 and v2 are from [0, nbnodes], better use getidv1_startat1() and getidv2_startat1()
  int v1, v2;   // The two extremities of an edge v1v2 or of an arc (v1,v2)
  
  float length; // should be set from the start

  double algo_cost;
  
  // return the extremity disctinc from v in O(1).
  int return_other_extrem(int v);

  void set_algo_cost(double v);

  float getDistance(); //returns this.length //added 
  int getidv1_startat0(); //added thomas
  int getidv2_startat0();//added thomas

   /******** Lemon structure ****/
   lemon::ListGraph::Edge LGU_name;
   lemon::ListDigraph::Arc LGD_name;
  
};


/***************************  C_node  *****************************/
class C_node{
public :
   int num;     // Number of the node
   float weight;
   float x,y;

   int VRP_demand; //added
   
   list <C_link*> L_adjLinks;

   //Test if j is a neighbour of i in O(degre(i))
   bool test_neighbour(int j);

   //Test if j is a successor of i in O(degre(i))
   bool test_successor(int j);

   //return EUC_2D distance to other node
   float getDistanceFrom_startat0(int idj);  //added thomas //ok verifié

   /******** Lemon structure ****/
   lemon::ListGraph::Node LGU_name;
   lemon::ListDigraph::Node LGD_name;

};


/**************************  C_Graph  ******************************/
class C_Graph{
public:

  C_Graph(){};

  bool directed;  // True if directed / False if undirected
  int nb_nodes;   // Number of nodes
  int nb_links;   // Number of links

  int VRP_capacity; //added

  float maxx,maxy,minx,miny;
  float lengthTSP(int i, int j);

  // Encoding of the graph by adjacence list, i.e. a vector of list of edges 
  vector <C_node> V_nodes;

  // Additional encoding: a vector on the edges (on pointers over edges)
  vector <C_link*> V_links;

  /******* VRP functions added ********/ // added thomas
  C_node* get_node_by_id_startat0(int id); // added thomas

  float get_distance_startat0(int node1, int node2);

  //prend tournée sans le sommet 0, numérotée de 1 à N (ex: {15, 16, 17, 19, 21, 29} )
  //renvoie le cout total de la tournée **en prenant en compte le sommet 0** ajouté à la volé !
  // le sommet 0 ne DOIT PAS  etre dans le vector route 
  float get_route_cost(vector<int> route); //added thomas
  //similaire à ci-dessus mais cette fois avec toutes les tournées d'un coup en entrée
  float get_VRP_cost(vector<vector<int>> tournees);
  //renvoie la capacité utilisée par cette tournée (somme des demandes des clients)
  float get_route_demand(vector<int> route);
  ///////////////////////////////
  //these are used when you want cycles that don't necessarily are from and to Depot
  // in this case you MUST add the node 0 in the route vector at the begining AND the end !!
  // for exemple a route would be : 0, 14, 24, 0
  float get_route_cost_notalwaysfromDepot(vector<int> route);
  float get_VRP_cost_notalwaysfromDepot(vector<vector<int>> tournees);
  /************************************/

  /*********************************************/
  /********* LEMON Structure *******************/
  lemon::ListGraph L_GU;
  lemon::ListDigraph L_GD;
  map<int, int> L_rtnmap; // map between the num of the ad hoc graph
                             // and the lemon id of a node
  
  void construct_Undirected_Lemon_Graph();
  void construct_Directed_Lemon_Graph();

  /*********************************************/
  /*********** INPUT-OUTPUT FILES **************/
  
  // Read a DIMACS file and store the corresponding graph in C_Graph
  void read_undirected_DIMACS(istream & fic);

  // Read a directed "gra" format file
  // and store the corresponding graph in C_Graph
  void read_directed_GRA(istream & in);

  // Read a complete undirected "tsp" format file
  // and store the corresponding graph in C_Graph
  void read_undirected_complete_TSP(istream & in);

  
  // Write a Graphviz File with the DOT format
  void write_dot_G(string InstanceName);

  // Write the node cloud in SVG format
  void write_SVG_node_cloud(string InstanceName);
  
  
  // Write a Graphviz File with the DOT format using an incidence vector of a stable set
  void write_dot_G_stableset(string InstanceName, vector<int> &stable);

  // Write a Graphviz File with the DOT format using a coloration vector
  void write_dot_G_color(string InstanceName, vector<int> &coloring);


  void write_dot_G_color_svg(string InstanceName, vector<int>& coloring); // added thomas
  
  // Write a Graphviz File with the DOT DIRECTED format using a induced subgraph
  void write_dot_directed_G_induced(string InstanceName, vector<int>& sol);

  // Write a SVG file format using a TSP tour
  void write_SVG_tour(string InstanceName, list<pair<int,int> >&sol);
  

  /*********************************************/
  /*********** ALGORITHMS **********************/

  // Return true if the directed graph induced by node subset sol is acyclic
  // Use a deep-first search algorithm in O(n+m)
  bool detect_circuit(vector<int>&sol);

  // Return true if the directed graph induced by node subset sol is acyclic
  // If it is the case, the list L will contain the nodes of a circuit
  // Use a deep-first search algorithm in O(n+m)
  bool return_circuit(vector<int>&sol, list<int>&L);

  // Calculates the shortest path tree (SPT) rooted in u
  // The out-parameter T is assumed to be allocated.
  // Return vector T as follows:  T[i]=-1 if i=u;
  // T[i]= its father in the SPT or -2 if i is unreachable from u
  // Return vector dist such that dist[i] is the shortest distance from u to i
  // Recall: The Dijkstra algorithm solves the single-source shortest
  // path problem when all arc lengths are non-negative. 
  void Directed_ShortestPathTree(int u, vector<int>& T, vector<float>& dist);

  // Calculates the shortest path from s to t
  // At the end, if P is empty, then t is unreachable
  // Return P as the node list of the shortest path
  double Directed_ShortestPath(int s, int t, list<int>& P);
  
  
  // Calculates the minimum cut in an undirected graph
  //   with the Nagamochi-Ibaraki algorithm.
  // Complexity in  O(nm+n^2log(n)) using Fibonacci heap
  // Return the value of the minimum cut and a node list W inducing the cut
  double Undirected_MinimumCut(list<int>& W);

  
};
#endif
