#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include"../../Graph/Graph.h"

#define epsilon 0.00001

using namespace std;

///////////////////////////////
///////////////////////////////

// MIP formulation for the Stable Set Problem
// Given a graph G=(V,E)

// sum_{i in V} x_i
// x_i + x_j <= 1  for every edge ij in E
// x_i in {0,1} for every node i in V

///////////////////////////////
///////////////////////////////

int main (int argc, char**argv){

  string name,nameext,nameextsol;
  int i,k;
  list<int>::const_iterator it;

  vector<int> sol;


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
  //////  CPLEX INITIALIZATION
  //////////////


  IloEnv   env;
  IloModel model(env);



  ////////////////////////
  //////  VAR
  ////////////////////////

  //IloNumVarArray x(env, G.nb_nodes, 0.0, 1.0, ILOINT);
  vector<IloNumVar> x;
  x.resize(G.nb_nodes);
  for(i = 0; i < G.nb_nodes; i++) {
    x[i]=IloNumVar(env, 0.0, 1.0, ILOINT);
    ostringstream varname;
    varname.str("");
    varname<<"x_"<<i;
    x[i].setName(varname.str().c_str());
  }


  //////////////
  //////  CST
  //////////////

  IloRangeArray CC(env);
  int nbcst=0;

  // Cst x_i + x_j \le 1 for every edges ij in E

  for (k=0;k<G.nb_links;k++){
      IloExpr cst(env);
      cst+=x[G.V_links[k]->v1]+x[G.V_links[k]->v2];
      CC.add(cst<=1);
      ostringstream cstname;
      cstname.str("");
      cstname<<"Cst_edge_"<<G.V_links[k]->v1<<"_"<<G.V_links[k]->v2;
      CC[nbcst].setName(cstname.str().c_str());
      nbcst++;
  }


  model.add(CC);


  //////////////
  ////// OBJ
  //////////////
  
  IloObjective obj=IloAdd(model, IloMaximize(env, 0.0));
  
  for (i=0;i<G.nb_nodes;i++)
    obj.setLinearCoef(x[i],G.V_nodes[i].weight);
 

  ///////////
  //// RESOLUTION
  //////////

  IloCplex cplex(model);

  // cplex.setParam(IloCplex::Cliques,-1);
  // cplex.setParam(IloCplex::Covers,-1);
  // cplex.setParam(IloCplex::DisjCuts,-1);
  // cplex.setParam(IloCplex::FlowCovers,-1);
  // cplex.setParam(IloCplex::FlowPaths,-1);
  // cplex.setParam(IloCplex::FracCuts,-1);
  // cplex.setParam(IloCplex::GUBCovers,-1);
  // cplex.setParam(IloCplex::ImplBd,-1);
  // cplex.setParam(IloCplex::MIRCuts,-1);
  // cplex.setParam(IloCplex::ZeroHalfCuts,-1);
  // cplex.setParam(IloCplex::MCFCuts,-1);
  // cplex.setParam(IloCplex::MIPInterval,1);
  // cplex.setParam(IloCplex::HeurFreq,-1);
  // cplex.setParam(IloCplex::ClockType,1);
  // cplex.setParam(IloCplex::RINSHeur,-1);


  cplex.exportModel("sortie.lp");


  if ( !cplex.solve() ) {
    env.error() << "Failed to optimize LP" << endl;
    exit(1);
  }

 
  env.out() << "Solution status = " << cplex.getStatus() << endl;
  env.out() << "Solution value  = " << cplex.getObjValue() << endl;


  vector<int>   solx;
  solx.resize(G.nb_nodes);
  for(i = 0; i < G.nb_nodes; i++) {
    solx[i]=cplex.getValue(x[i]);
  }  


  //////////////
  //////  CPLEX's ENDING
  //////////////

  env.end();

  //////////////
  //////  OUTPUT
  //////////////

  ofstream ficsol(nameextsol.c_str());
  
  for(i = 0; i < G.nb_nodes; i++) 
    ficsol<<solx[i]<<" ";

  ficsol.close();


  return 0;
}
