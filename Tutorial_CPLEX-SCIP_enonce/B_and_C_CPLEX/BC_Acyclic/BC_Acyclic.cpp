#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include"../../Graph/Graph.h"

#define epsilon 0.00001

using namespace std;


//#define OUTPUT


///////////////////////////////
///////////////////////////////

// Non-Compact MIP formulation for the Acyclic Induced Subgraph Problem
// Given a directed graph G=(V,A)

// sum_{i in V} x_i
// sum_{i\in V(C)} x_i<= |C|-1 for all circuit C of G
// x_i in {0,1} for every node i in V

///////////////////////////////
///////////////////////////////


///////////////////////////////
////// SEPARATION FCT ////////
/////////////////////////////

// declaration of the functions that are dispatched outstide this file
void find_ViolatedAcyclicCst_INTEGER(IloEnv env, C_Graph & G,  vector<IloNumVar>& x, vector<float>&fracsol, list<IloRange> & L_ViolatedCst);

void find_ViolatedAcyclicCst(IloEnv env, C_Graph & G,  vector<IloNumVar>& x, vector<float>&fracsol, list<IloRange> & L_ViolatedCst);

// Cplex callback that needs to be in the same file as the main one

// Necessary inequalities
ILOLAZYCONSTRAINTCALLBACK2(LazyCircuitSeparation,
			   C_Graph &, G,			 
			   vector<IloNumVar>&,x
		    ){
  #ifdef OUTPUT
  cout<<"*********** Lazy separation Callback *************"<<endl;
  #endif

  int i;
  vector<float> fracsol;
  list<IloRange> L_ViolatedCst;
  fracsol.resize(G.nb_nodes);
      
  for (i=0;i<G.nb_nodes;i++)
    fracsol[i]= getValue(x[i]);
 
  /* Separation of Circuit inequalities */

  L_ViolatedCst.clear();
  find_ViolatedAcyclicCst_INTEGER(getEnv(),G,x,fracsol, L_ViolatedCst);
  
  #ifdef OUTPUT
  if (L_ViolatedCst.empty()) cout<<"No Cst found"<<endl;
  #endif
  
  while (!L_ViolatedCst.empty()){
    #ifdef OUTPUT
      cout << "Adding constraint : " << L_ViolatedCst.front() << endl;
    #endif
    add(L_ViolatedCst.front(),IloCplex::UseCutForce); //UseCutPurge);
    L_ViolatedCst.pop_front();
  }

}


// Usefull inequalities (here are the same as the necessary ones
ILOUSERCUTCALLBACK2(UsercutCircuitSeparation,
			   C_Graph &, G,
			   vector<IloNumVar>&,x
		    ){
  #ifdef OUTPUT
  cout<<"********* UserCut separation Callback *************"<<endl;
  #endif


  int i;
  vector<float> fracsol;
  list<IloRange> L_ViolatedCst;
  fracsol.resize(G.nb_nodes);
      
  for (i=0;i<G.nb_nodes;i++)
    fracsol[i]= getValue(x[i]);
 
  /* Separation of Circuit inequalities */
  
  L_ViolatedCst.clear();
  find_ViolatedAcyclicCst(getEnv(),G,x,fracsol, L_ViolatedCst);
  
  #ifdef OUTPUT
  if (L_ViolatedCst.empty()) cout<<"No Cst found"<<endl;
  #endif
  
  while (!L_ViolatedCst.empty()){
    #ifdef OUTPUT
      cout << "Adding constraint : " << L_ViolatedCst.front() << endl;
    #endif
    add(L_ViolatedCst.front(),IloCplex::UseCutForce); //UseCutPurge);
    L_ViolatedCst.pop_front();
  }

}



//////////////////////////////////////
/////  CHECK SOLUTION FEASABILITY ////
//////////////////////////////////////

// Check if an integer solution (can be obtained by Cplex by heuristic
// or within a B&B node
ILOINCUMBENTCALLBACK2(CheckSolFeas,
		      C_Graph &, G,
		      vector<IloNumVar>&,x
		      ){

#ifdef OUTPUT
  cout << "******* Check Feasability ********" << endl;
#endif

  int i;
  vector<float> fracsol;
  fracsol.resize(G.nb_nodes);

  vector<int> sol;
  sol.resize(G.nb_nodes);
  for (i=0;i<G.nb_nodes;i++){
    fracsol[i]= getValue(x[i]);
    if (fracsol[i]<=epsilon) sol[i]=0;
    else sol[i]=1;
  }

  if (G.detect_circuit(sol)){
    reject();
    #ifdef OUTPUT
    cout<<"Solution Reject"<<endl;
    #endif
  }
  #ifdef OUTPUT
  else
    cout<<"Solution accepted"<<endl;   
  #endif
}


//////////////////////////////////////
/////   PRIMAL HEURISTIC          ////
//////////////////////////////////////

class sortingNode{
public:
  int v;
  double val;

  bool operator<(sortingNode& u){
    return (val>u.val);
  }
};


// Try to produce a heuristic solution from fractional solution
ILOHEURISTICCALLBACK2(PrimalHeuristic,
		      C_Graph &, G,
		      vector<IloNumVar>&,x
		      ){

  
  #ifdef OUTPUT
  cout << "******* Primal Heuristic ********" << endl;  
  #endif
  
  int i, cpt;
  list<sortingNode> L;
  list<sortingNode>::const_iterator it;

  L.empty();
  for (i=0;i<G.nb_nodes;i++){
    sortingNode u;
    u.v=i;
    u.val=getValue(x[i]);
    L.push_back(u);
  }
  L.sort();

  vector<int> sol;
  sol.resize(G.nb_nodes);
  for (i=0;i<G.nb_nodes;i++)
    sol[i]=0;

  // The following loop has an awfully complexity of O(n^2+nm)
  // There is another possibility with a code similar to
  // the Kruskall Union-Find structure in O(n+m)
  // Since this heurisitic is often used in the code
  // it will be really mandatory to give her the smallest complexity
  // or to encode a this place another cheaper heuristic
  cpt=0;
  for (it=L.begin();it!=L.end();it++){
    sol[it->v]=1;
    if (G.detect_circuit(sol))
      sol[it->v]=0;
    else
      cpt++;
  }


  IloIntVarArray solx(getEnv(),G.nb_nodes,0.0,1.0);
  IloNumArray valx(getEnv(),G.nb_nodes);

  for (i=0;i<G.nb_nodes;i++){
    solx[i]=x[i];
    valx[i]=sol[i];    
  }
  

  //cout<<"Heuristic solution found: "<<value<<endl;
  setSolution(solx,valx,cpt);

}




///////////////////////////////
////////// MAIN ///////////////
//////////////////////////////

int main (int argc, char**argv){
  
  string name, nameext, nameextsol;
  int i,k;

  vector<int> sol;


  //////////////
  //////  DATA
  //////////////

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" <GRA file name>   (without .gra)"<<endl; 
    return 1;
  }
  
  name=argv[1];
  nameext=name+".gra";
  nameextsol=name+".acycl";


  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" not found"<<endl;
    return 1;
  }

  C_Graph G;

  G.read_directed_GRA(fic);

  fic.close();


  //////////////
  //////  CPLEX INITIALIZATION
  //////////////


  IloEnv   env;
  IloModel model(env);



  ////////////////////////
  //////  VAR
  ////////////////////////

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
  ////// OBJ
  //////////////
  
  IloObjective obj=IloAdd(model, IloMaximize(env, 0.0));
  
  for (i=0;i<G.nb_nodes;i++)
    obj.setLinearCoef(x[i],G.V_nodes[i].weight);
 

  //////////////
  //////  CST
  //////////////

  IloRangeArray CC(env);
  int nbcst=0;
  list<C_link*>::const_iterator it;


  // 2-node circuit inequalities
  // x_i + x_j <= 1 for all circuit ( (i,j) , (j,i) ) if there exists some

  for (i=0;i<G.nb_nodes;i++){
    for (it=G.V_nodes[i].L_adjLinks.begin();it!=G.V_nodes[i].L_adjLinks.end();it++){
      if (G.V_nodes[(*it)->v2].test_successor(i)){
	IloExpr c1(env);
	c1+=x[i]+x[(*it)->v2];
	CC.add(c1<=1);
	ostringstream nomcst;
	nomcst.str("");
	nomcst<<"Cst_TwoCirc_"<<i<<"_"<<(*it)->v2;
	CC[nbcst].setName(nomcst.str().c_str());
	nbcst++;
      }
    }
  }


  model.add(CC);

  // And Finally
  IloCplex cplex(model);

  
  /// ADD SEPARATION CALLBACK
  cplex.use(LazyCircuitSeparation(env,G,x));
  cplex.use(UsercutCircuitSeparation(env,G,x));




  //////////////
  ////// ADDITIONNAL FUNCTION AND PARAMETERS
  /////////////
   
  // ADD CHECK SOLUTION FEASABILITY

  cplex.use(CheckSolFeas(env,G,x));

    // ADD A PRIMAL HEURISTIC
  // tente d'arrondir les solutions pour voir
  cplex.use(PrimalHeuristic(env,G,x)); 


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


  
  ///////////
  //// RESOLUTION
  //////////


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
