#include <ilcplex/ilocplex.h>
#include <vector>
#include <list>

#include "Graph.h"

using namespace std;


class Cplex_pricing_algo{
 public:

  C_Graph *G;
  
  IloEnv   env;
  IloModel model;
  IloObjective obj;
  IloCplex cplex;
      
  IloNumVarArray x;


  list<list<int> > *L_notequal; // Pointeur on a list of stable sets that
                                // have not to be found as solutions

  void initialization(C_Graph *G);
  

  void set_objective_coefficient(const vector<double>& obj_coeff);

  void aff_forbidden_stable_set();

  void set_forbidden_stable_set(list<list<int> >* LL_notequal);
  
  // Launch Cplex solver and get back an optimal stable set 
  bool find_stableset(list<int>& stable, double& objvalue);


};
