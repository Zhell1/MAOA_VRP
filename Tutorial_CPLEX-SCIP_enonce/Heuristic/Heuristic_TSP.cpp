#include <iostream>
#include <iomanip>
#include <cstring>
#include <sstream>
#include<ctime>

#include"../Graph/Graph.h"


using namespace std;


//#define OUTPUT 

///////////////////////////////
///////////////////////////////

// Greedy Heuristic + 2-OPT descent for the TSP problem
// Given a complete undirected graph G=(V,E)

// Greedy Algorithm
// S <- 0
// While |S|<n
//   Found the closest node from S
//   Add it to S

// 2-OPT descent from the greedy solution
// For NB-ITER iteration
//    Randomly choose one edge ij of the solution
//    Randomly choose one non-incident edge kl of the solution
//    Stochatic test for accepting ik jl instead of ij kl

///////////////////////////////
///////////////////////////////


void greedy_heuristic(C_Graph &G, vector<int>& sol){

  int i,k;
  
  vector<int> added;
  added.resize(G.nb_nodes);    
  for (i=0;i<G.nb_nodes;i++) added[i]=0;
  
  int curr=0;
  float next;
  int nexti;
  
  sol[0]=0;
  added[0]=1;
  k=1;
  while (k<G.nb_nodes){

    next=10e6;
    for (i=0;i<G.nb_nodes;i++)
      if (added[i]==0)
	if (next>G.lengthTSP(curr,i)){
	  next=G.lengthTSP(curr,i);
	  nexti=i;
	}
    sol[k]=nexti;
    added[nexti]=1;
    curr=nexti;
    k++;
  }

}

void TwoOptDescent(C_Graph &G, double& curr_length, vector<int>& curr_sol, double& best_length, list<pair<int,int> >& best_sol, int NB_ITER){
  
  int k;
  int nbiter;
  int k1,k2;
  int jm,j,jp,lm,l,lp;
  bool found;
  double v1,v2;
  
  for (nbiter=0;nbiter<NB_ITER;nbiter++){

    k1=rand()%G.nb_nodes;
    k2=rand()%(G.nb_nodes-1)+1;
    k2=(k1+k2)%G.nb_nodes;

    if (k1==0) jm=curr_sol[G.nb_nodes-1];
       else jm=curr_sol[(k1-1)%G.nb_nodes];
    j=curr_sol[k1];
    jp=curr_sol[(k1+1)%G.nb_nodes];
    if (k2==0) lm=curr_sol[G.nb_nodes-1];
       else lm=curr_sol[(k2-1)%G.nb_nodes];
    l=curr_sol[k2];
    lp=curr_sol[(k2+1)%G.nb_nodes];
    
    if (k2==(k1+1)%G.nb_nodes){
      v1=G.lengthTSP(jm,j) + G.lengthTSP(l,lp);
      v2=G.lengthTSP(jm,l) + G.lengthTSP(j,lp);
    }
    else
      if (k1==(k2+1)%G.nb_nodes){
	v1=G.lengthTSP(lm,l) + G.lengthTSP(j,jp);
	v2=G.lengthTSP(lm,j) + G.lengthTSP(l,jp);
      }
      else{
	v1=G.lengthTSP(jm,j) + G.lengthTSP(j,jp) + G.lengthTSP(lm,l) + G.lengthTSP(l,lp) ;
	v2=G.lengthTSP(jm,l) + G.lengthTSP(l,jp) + G.lengthTSP(lm,j) + G.lengthTSP(j,lp);
      }

    if (v1>v2){
      curr_length=curr_length-v1+v2;
      curr_sol[k1]=l;
      curr_sol[k2]=j;	    
    }
    else{
      if (rand()%1000<1){
    	curr_length=curr_length-v1+v2;
  	curr_sol[k1]=l;
  	curr_sol[k2]=j;	    
      }      
    }

    double test=0;
    for(k=1; k<G.nb_nodes;k++)
      test+=G.lengthTSP(curr_sol[k-1],curr_sol[k]);
    test+=G.lengthTSP(curr_sol[G.nb_nodes-1],curr_sol[0]);
    
    if (best_length>curr_length){
      best_sol.clear();
      for(k=1; k<G.nb_nodes;k++) {
 	best_sol.push_back(make_pair(curr_sol[k-1],curr_sol[k]));
      }  
      best_sol.push_back(make_pair(curr_sol[G.nb_nodes-1],curr_sol[0]));
      best_length=curr_length;    
    }
    #ifdef OUTPUT
    cout<<"Curr sol: "<<curr_length<<" /" <<best_length<<endl;
    #endif
  }

}

int main (int argc, char**argv){

string name, nameext, nameextsol;
  int i,k,NB_ITER;
  list<pair<int,int> >::const_iterator itp;
  
  clock_t time_initial, /* Time initial en micro-secondes */
          time_final;   /* Time final en micro-secondes */
  float   time_cpu;     /* Time total en secondes */

  //////////////
  //////  DATA
  //////////////

  if(argc!=3){
    cerr<<"usage: "<<argv[0]<<" <TSP file name (without .tsp)> <nb_iteration>"<<endl;
    cerr<<"       0: only greedy / nb>0: greedy+TwoOptDescent on nb iterations"<<endl;
    return 1;
  }

  name=argv[1];
  nameext=name+".tsp";
  nameextsol=name+".tour";
  NB_ITER=atoi(argv[2]);

  
  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" not found"<<endl;
    return 1;
  }

  C_Graph G;

  G.read_undirected_complete_TSP(fic);

  fic.close();


  //////////////
  //////  GREEDY HEURISTIC
  //////////////
  time_initial = clock ();

  list<pair<int,int> >best_sol;
  best_sol.clear();

  vector<int> curr_sol;
  curr_sol.resize(G.nb_nodes);

  greedy_heuristic(G,curr_sol);
  
  double curr_length=0;
  for(k=1; k<G.nb_nodes;k++) {
    curr_length+=G.lengthTSP(curr_sol[k-1],curr_sol[k]);
    best_sol.push_back(make_pair(curr_sol[k-1],curr_sol[k]));
  }
  curr_length+=G.lengthTSP(curr_sol[G.nb_nodes-1],curr_sol[0]);
  best_sol.push_back(make_pair(curr_sol[G.nb_nodes-1],curr_sol[0]));
  double best_length=curr_length;

  cout<<"Greedy Tour of value : "<<best_length<<endl;


  //////////////
  //////  2-OPT descent
  //////////////

  TwoOptDescent(G, curr_length, curr_sol, best_length, best_sol,NB_ITER);
  cout<<"Best tour value after "<<NB_ITER<<" 2-OPT: "<<best_length<<endl;

  
  //////////////
  //////  OUTPUT
  //////////////
  
  time_final = clock ();
  time_cpu = (time_final - time_initial) * 1e-6;
  
  

  ofstream ficsol(nameextsol.c_str());
  
  for(itp = best_sol.begin(); itp!=best_sol.end();itp++) {
    ficsol<<itp->first<<" "<<itp->second<<endl;
  }
 
  ficsol.close();

  cout<<"Tour found of value : "<<best_length<<endl;
  cout<<"Within "<<time_cpu<<" seconds"<<endl;

  return 0;
}
