#include <iostream>
#include <iomanip>
#include <cstring>
#include <sstream>
#include<ctime>

#include"../Graph/Graph.h"


#define MAX_ITER 10000
#define MAX_ITER_IMPROV 1000

using namespace std;


//#define OUTPUT 

///////////////////////////////
///////////////////////////////

// Greedy Heuristic + Iterative Stochastic Descent
// Given a graph G=(V,E)

// Greedy Algorithm
// For every node 
//    Try to add the node to a set S if S is still a stable set

// Simple Iterative Descent

// Neighborhood 1: delete a node
// Neighborhood 2: add a nodeS

// For MAX_ITER_DESCENT iterations
//  While no improvement after MAX_ITER_IMPROVNT
//   Randomly choose a neighborhood and a node to apply it
//   Stochastic choice to keep the produced solution

///////////////////////////////
///////////////////////////////



int main (int argc, char**argv){

  string  name, nameext, nameextsol;
  int i,k;
  list<C_link*>::const_iterator itl;
  
  clock_t time_initial, /* Time initial en micro-secondes */
          time_final;   /* Time final en micro-secondes */
  float   time_cpu;     /* Time total en secondes */

  //////////////
  //////  DATA
  //////////////

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" <DIMACS file name>   (without .dim)"<<endl; 
    return 1;
  }

  name=argv[1];
  nameext=name+".dim";
  nameextsol=name+".stable";


  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" not found"<<endl;
    return 1;
  }

  C_Graph G;

  G.read_undirected_DIMACS(fic);

  fic.close();


  //////////////
  //////  GREEDY HEURISTIC
  //////////////
  time_initial = clock ();

  int val_best_sol;
  vector<int> best_sol;
  best_sol.resize(G.nb_nodes);

  int val_greedy_sol;
  vector<int> greedy_sol;
  greedy_sol.resize(G.nb_nodes);
  
  vector<int> V_nbvois;
  V_nbvois.resize(G.nb_nodes);
  
  for (i=0;i<G.nb_nodes;i++){
    greedy_sol[i]=0;
    V_nbvois[i]=0;
  }
  
  val_greedy_sol=0;
  for (i=0;i<G.nb_nodes;i++){
    if (V_nbvois[i]==0){
      greedy_sol[i]=1;
      val_greedy_sol++;
      for (itl=G.V_nodes[i].L_adjLinks.begin();itl!=G.V_nodes[i].L_adjLinks.end();itl++)
	V_nbvois[(*itl)->return_other_extrem(i)]++;
    }      
  }

  for (i=0;i<G.nb_nodes;i++) best_sol[i]=greedy_sol[i];
  val_best_sol=val_greedy_sol;

  cout<<"Solution gloutonne : "<<val_best_sol<<endl;
					     
  //////////////
  //////  META-HEURISTIC
  //////////////

  srand(time(NULL));

  int val_curr_sol;
  vector<int> curr_sol;
  curr_sol.resize(G.nb_nodes);

  int nb;
  int cand;
  int cpt_non_improv;
  for (nb=0;nb<MAX_ITER;nb++){
    #ifdef OUTPUT
    cout<<"Iteration: "<<nb<<endl;
    #endif
    
    for (i=0;i<G.nb_nodes;i++) V_nbvois[i]=0;
    
    for (i=0;i<G.nb_nodes;i++){
      curr_sol[i]=greedy_sol[i];
      if (curr_sol[i]==1){
	for (itl=G.V_nodes[i].L_adjLinks.begin();itl!=G.V_nodes[i].L_adjLinks.end();itl++)
	  V_nbvois[(*itl)->return_other_extrem(i)]++;
      }
    }    
    val_curr_sol=val_greedy_sol;
    
    cpt_non_improv=0;

    while (cpt_non_improv<MAX_ITER_IMPROV){

      cpt_non_improv++;

      cand=rand()%G.nb_nodes;
      
      if ((rand()%2==0)&&(curr_sol[cand]==0)){ // Adding attempt
        #ifdef OUTPUT
	cout<<"Try to add "<<cand<<endl;
        #endif

	itl=G.V_nodes[cand].L_adjLinks.begin();
	while (   (itl!=G.V_nodes[cand].L_adjLinks.end())
		  && (curr_sol[(*itl)->return_other_extrem(cand)]==0) )
	  itl++;
	if (itl==G.V_nodes[cand].L_adjLinks.end()){
	  curr_sol[cand]=1;
	  val_curr_sol++;
	  for (itl=G.V_nodes[cand].L_adjLinks.begin();itl!=G.V_nodes[cand].L_adjLinks.end();itl++)
	    V_nbvois[(*itl)->return_other_extrem(cand)]++;
	}
      }
      else{  // deleting attempt
	if ((curr_sol[cand]==1)&&(rand()%100<20)) {
          #ifdef OUTPUT
          cout<<"Delete "<<cand<<endl;
          #endif
	  curr_sol[cand]=0;
	  for (itl=G.V_nodes[cand].L_adjLinks.begin();itl!=G.V_nodes[cand].L_adjLinks.end();itl++)
	    V_nbvois[(*itl)->return_other_extrem(cand)]--;
	  val_curr_sol--;	  
	}
      }
  
      if (val_curr_sol>val_best_sol){
	for (i=0;i<G.nb_nodes;i++) best_sol[i]=curr_sol[i];
	val_best_sol=val_curr_sol;
	cpt_non_improv=0;
      }
      #ifdef OUTPUT
      cout<<"   Value : "<<val_curr_sol<<" / "<<val_best_sol<<" "<<endl;
      #endif      
    }

  }
  #ifdef OUTPUT
  cout<<"END OF ITERATION"<<endl;
  #endif
  
  //////////////
  //////  OUTPUT
  //////////////
  
  time_final = clock ();
  time_cpu = (time_final - time_initial) * 1e-6;
  
  
  int cpt;

  bool stable=true;
  i = 0;
  while ((i < G.nb_nodes) && (stable)) {
    if (best_sol[i]==1) {
      itl=G.V_nodes[i].L_adjLinks.begin();
      while ((itl!=G.V_nodes[i].L_adjLinks.end()) && (stable)){
  	if (best_sol[(*itl)->return_other_extrem(i)]==1)
  	  stable=false;	
  	itl++;
      }
    }
    i++;
  }
  if (!stable )
    cout<<"False solution!"<<endl;
  else
    cout<<"Solution found"<<endl;

  
  ofstream ficsol(nameextsol.c_str());
  
  cpt=0;
  for(i = 0; i < G.nb_nodes; i++) {
    if (best_sol[i]==1) cpt++;
    ficsol<<best_sol[i]<<" ";
  }
 
  ficsol.close();

  cout<<"Stable set found: "<<cpt<<" nodes"<<endl;
  cout<<"Within "<<time_cpu<<" seconds"<<endl;

  return 0;
}
