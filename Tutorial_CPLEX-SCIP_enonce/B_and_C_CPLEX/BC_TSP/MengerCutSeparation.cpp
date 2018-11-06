#include <ilcplex/ilocplex.h>
#include <vector>
#include"../../Graph/Graph.h"

#define epsilon 0.01

using namespace::std;


//Find a violated Cut when x is integer (erf
bool  find_ViolatedMengerCutCst_INTEGER(IloEnv env, C_Graph & G,  vector<vector<IloNumVar> >& x,  IloRange & ViolatedCst){

  int u,v, start;
  list<int>::const_iterator it;
  list<C_link *>::iterator ita;
  vector<int> marq;
  list<int> W;
  vector<int> V_W;

  W.empty();
  V_W.resize(G.nb_nodes);
  
  marq.resize(G.nb_nodes);
  for (u=0;u<G.nb_nodes;u++) marq[u]=0;
  
  start=rand()%G.nb_nodes;
  u=start;
  v=-1;
  
  do {
    ita=G.V_nodes[u].L_adjLinks.begin();

    while (((*ita)->return_other_extrem(u)==v) || ((*ita)->algo_cost!=1))
      ita++;
    v=u;
    u=(*ita)->return_other_extrem(u);
    W.push_back(u);
  } while (u!=start);
 

  if (W.size()!=G.nb_nodes){
      
    IloExpr expr(env);
    for (u=0;u<G.nb_nodes;u++)  V_W[u]=0;
    for (it=W.begin();it!=W.end();it++) V_W[*it]=1;

    // cout<<"[integer] Set W found :";
    // for (it=W.begin();it!=W.end();it++)
    //   cout<<*it<<" ";
    // cout<<endl;
    
    for (it=W.begin();it!=W.end();it++)
      for (v=0;v<G.nb_nodes;v++)
    	if (V_W[v]==0) expr+=x[*it][v];

      
    ViolatedCst=IloRange(expr >= 2);
    return true;
  }
  else
    return false;

}






//Find a violated "Menger" Cut by a separation algorithm
bool  find_ViolatedMengerCutCst(IloEnv env, C_Graph & G,  vector<vector<IloNumVar> >& x,  IloRange & ViolatedCst){

  int i,j;
  list<int> W;
  list<int>::const_iterator it;
  vector<int> V_W;
  float test;

  V_W.resize(G.nb_nodes);

  // Find a minimum cut


  test=G.Undirected_MinimumCut(W);

  //cout<<"test = "<<test<<endl;
  
  if (test<2-epsilon) {
    // Found a violated inequality 
      
    IloExpr expr(env);
    for (i=0;i<G.nb_nodes;i++)  V_W[i]=0;
    for (it=W.begin();it!=W.end();it++) V_W[*it]=1;

    // cout<<"Set W found :";
    // for (it=W.begin();it!=W.end();it++)
    //   cout<<*it<<" ";
    // cout<<endl;
    
    for (it=W.begin();it!=W.end();it++)
      for (j=0;j<G.nb_nodes;j++)
    	if (V_W[j]==0) expr+=x[*it][j];

      
    ViolatedCst=IloRange(expr >= 2);
    return true;
  }
  else
    return false;

}



