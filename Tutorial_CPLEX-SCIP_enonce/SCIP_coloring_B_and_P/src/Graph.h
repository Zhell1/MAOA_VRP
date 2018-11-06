#ifndef Graph_H
#define Graph_H

#include <vector>
#include <list>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>

#include <math.h>
#include <unistd.h>

using namespace std;

/****************************  C_edge  *******************************/
class C_link{
public:
  int num;      // Number of the edge
  int v1, v2;   // The two extremities of an edge v1v2 or of an arc (v1,v2)
  float weight;
  
  // return the extremity disctinc from v in O(1).
  int return_other_extrem(int v);
  
};


/***************************  C_node  *****************************/
class C_node{
public :
   int num;     // Number of the node
   float weight;
   
   list <C_link*> L_adjLinks;

   //Test if j is a neighbour of i in O(degre(i))
   bool test_neighbour(int j);

   //Test if j is a successor of i in O(degre(i))
   bool test_successor(int j);

};


/**************************  C_Graph  ******************************/
class C_Graph{
public:

  bool directed;  // True if directed / False if undirected
  int nb_nodes;   // Number of nodes
  int nb_links;   // Number of links

  // Encoding of the graph by adjacence list, i.e. a vector of list of edges 
  vector <C_node> V_nodes;

  // Additional encoding: a vector on the edges (on pointers over edges)
  vector <C_link*> V_links;

  /*********************************************/
  /*********** ALGORITHMS **********************/

  bool detect_cycle(vector<int>&sol);

  

  /*********************************************/
  /*********** INPUT-OUTPUT FILES **************/
  
  // Read a DIMACS file and store the corresponding graph in C_Graph
  void read_undirected_DIMACS(istream & fic);

  // Read a TSP file and store the corresponding graph in C_Graph
  void read_undirected_TSP(istream & fic);
  
  // Read a directed "gra" format file and store the corresponding graph in C_Graph
  void read_directed_GRA(istream & in);

  // Write a Graphviz File with the DOT format
  void write_dot_G(string InstanceName);
  
  // Write a Graphviz File with the DOT format using an incidence vector of a stable set
  void write_dot_G_stableset(string InstanceName, vector<int> &stable);

  // Write a Graphviz File with the DOT format using a coloration vector
  void write_dot_G_color(string InstanceName, vector<int> &coloring);
  
};
#endif
