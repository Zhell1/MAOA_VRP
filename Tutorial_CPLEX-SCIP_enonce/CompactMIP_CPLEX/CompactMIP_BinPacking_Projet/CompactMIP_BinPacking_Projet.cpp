#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include"../../Graph/Graph.h"
#include "./VRPfileParser.h"

#define epsilon 0.00001

using namespace std;

///////////////////////////////
///////////////////////////////

// MIP formulation for the BinPacking problem
// Given a graph G=(V,E)

///////////////////////////////
///////////////////////////////

int main (int argc, char**argv){

  string name,nameext,nameextsol;
  int i,k,l,u,e;
  list<int>::const_iterator it;

  vector<int> sol;

/*********************
   //old version
  //////////////
  //////  DATA
  //////////////

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" <DIMACS file name>   (without .dim)"<<endl; 
    return 1;
  }

  name=argv[1];
  nameext=name+".dim";
  nameextsol=name+".color";

  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" not found"<<endl;
    return 1;
  }

  C_Graph G;

  G.read_undirected_DIMACS(fic);

  fic.close();
**********************/


  //////////////
  //////  DATA
  //////////////

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" filename.vrp"<<endl; 
    return 1;
  }

  string filename = argv[1];

  C_Graph* G = parseVRPfile(filename);

  //C_node* cnode = graph_ptr->get_node_by_id(1);    //exemple d'utilisation
  //(*G).nb_nodes;    // G->nb_nodes;                //both mean the same here

  C_node* cnode  = (*G).get_node_by_id(1);


  //////////////
  //////  CPLEX INITIALIZATION
  //////////////


  IloEnv   env;
  IloModel model(env);


  ////////////////////////
  //////  VAR
  ////////////////////////

  //K borne sup sur la coloration
  int K = (*G).nb_nodes; // K= |V|
  //w
  vector<IloNumVar> w;
  w.resize(K);
  //w_l var binaire
  for(l = 0; l < K; l++) {
    w[l] = IloNumVar(env, 0.0, 1.0, ILOINT);
    ostringstream varname;
    varname.str("");
    varname<<"w_"<<l;
    w[l].setName(varname.str().c_str());
  }
  //x
  vector<vector<IloNumVar> > x;
  x.resize((*G).nb_nodes);
  //x_u vecteur binaire à K dimensions
  for(u = 0; u < (*G).nb_nodes; u++) {
    vector<IloNumVar> x_u;
    x_u.resize(K);
    //binaire
    for(l = 0; l < K; l++) {
      x_u[l] = IloNumVar(env, 0.0, 1.0, ILOINT); 
      ostringstream varname;
      varname.str("");
      varname<<"x_"<<u<<"_"<<l;
      x_u[l].setName(varname.str().c_str());
    }
    x[u] = x_u;
  }

  //////////////
  //////  CST
  //////////////

  //premiere contrainte

  IloRangeArray CC(env);
  int nbcst=0;
  //for every node u in V
  for (u = 0; u < (*G).nb_nodes; u++){
    //sum{l= 1 to K} x_u_l = 1 
    IloExpr cst(env);
    for (l = 0; l < K; l++){
      cst+=x[u][l];
    }
    CC.add(cst==1);
    ostringstream cstname;
    cstname.str("");
    cstname<<"Cst_1colorPerVertex_"<<u;
    CC[nbcst].setName(cstname.str().c_str());
    nbcst++;
  }

  //deuxieme contrainte

  //for every edge
  for (e = 0; e < (*G).nb_links; e++){
    int u = (*G).V_links[e]->v1;
    int v = (*G).V_links[e]->v2;
    //pour tout l dans 0 à K
    for(l = 0; l < K; l++) {
      IloExpr cst(env);
      cst = x[u][l] + x[v][l] - w[l];
      CC.add(cst<=0);
      ostringstream cstname;
      cstname.str("");
      cstname<<"Cst_arete_"<<e;
      CC[nbcst].setName(cstname.str().c_str());
      nbcst++;
    }
  }

  model.add(CC);


  //////////////
  ////// OBJ
  //////////////
  
  IloObjective obj=IloAdd(model, IloMinimize(env, 0.0));
  
  for (l = 0; l < K ; l++){
    obj.setLinearCoef(w[l],1);
  }
 
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


/*************************** HERE WE ARE ********************************/
//TODO
  /*
parcourir X_u_l, toruver la couleur activée et la mettre dans le vec solution

  */

  vector<int>   solw;
  solw.resize(K);
  for(u = 0; u < (*G).nb_nodes; u++) {
    int color = 0;
    for(l=0; l < K; l++) {
      if(cplex.getValue(x[u][l]) == 1) {
        color = l;
        break;
      }
    }
    solw[u] = color;
  }  


  //////////////
  //////  CPLEX's ENDING
  //////////////

  env.end();

  //////////////
  //////  OUTPUT
  //////////////

  ofstream ficsol(nameextsol.c_str());
  
  for(i = 0; i < (*G).nb_nodes; i++) 
    ficsol<<solw[i]<<" ";

  ficsol.close();

  return 0;
}
