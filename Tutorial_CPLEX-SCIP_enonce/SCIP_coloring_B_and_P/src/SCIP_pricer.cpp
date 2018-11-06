#include "SCIP_pricer.h"
#include "scip/cons_linear.h"
#include <map>
#include <vector>
#include <iostream>

//#define OUTPUT_PRICER


using namespace std;
using namespace scip;


/** Constructs the pricer object with the data needed
 *
 *  An alternative is to have a problem data class which allows to access the data.
 */
ObjPricerColoring::ObjPricerColoring(
   SCIP*                                scip,          /**< SCIP pointer */
   const char*                         pp_name,      /**< name of pricer */  
   C_Graph*                             GG,
C_master_coloring*                      MM
   ):
  ObjPricer(scip, pp_name, "Finds stable set with negative reduced cost.", 0, TRUE)
{
  
  G=GG;  // save a pointer on data graph
  M=MM;
  
  A_cplex.initialization(G);
 
}


/** Destructs the pricer object. */
ObjPricerColoring::~ObjPricerColoring()
{
  cout<<"Destructeur du pricer"<<endl;
}


/** initialization method of variable pricer (called after problem was transformed)
 *
 *  Because SCIP transformes the original problem in preprocessing, we need to get the references to
 *  the variables and constraints in the transformed problem from the references in the original
 *  problem.
 */
SCIP_DECL_PRICERINIT(ObjPricerColoring::scip_init)
{
  int i;
  for (i = 0; i < G->nb_nodes; i++){
    SCIPgetTransformedCons(scip, M->V_node_ineq[i], &(M->V_node_ineq)[i]);
   }

   return SCIP_OKAY;
}

/** Pricing of additional variables if LP is feasible.
 *
 *  - get the values of the dual variables you need
 *  - construct the reduced-cost arc lengths from these values
 *  - find the shortest admissible tour with respect to these lengths
 *  - if this tour has negative reduced cost, add it to the LP
 *
 *  possible return values for *result:
 *  - SCIP_SUCCESS    : at least one improving variable was found, or it is ensured that no such variable exists
 *  - SCIP_DIDNOTRUN  : the pricing process was aborted by the pricer, there is no guarantee that the current LP solution is optimal
 */
SCIP_DECL_PRICERREDCOST(ObjPricerColoring::scip_redcost)
{
   SCIPdebugMsg(scip, "call scip_redcost ...\n");

   /* set result pointer, see above */
   *result = SCIP_SUCCESS;

   /* call pricing routine */
   coloring_pricing(scip);

   return SCIP_OKAY;
} 





// Perform pricing
void ObjPricerColoring::coloring_pricing(
   SCIP*                 scip       /**< SCIP data structure */
   )
{
  #ifdef OUTPUT_PRICER
  cout<<"**************PRICER************"<<endl;
  #endif
  

  
  int i;
  vector<SCIP_Real> dual_cost(G->nb_nodes);
  
  for (i = 0; i < G->nb_nodes; i++)
    dual_cost[i]= SCIPgetDualsolLinear(scip, M->V_node_ineq[i]);
    
  #ifdef OUTPUT_PRICER
  cout<<"dual solution"<<endl;
  for (i = 0; i < G->nb_nodes; i++){
    cout<<"Cte_"<<i<<"="<<dual_cost[i]<<" ";
  }
  cout<<endl;
  #endif

	  
  /* compute shortest length restricted tour w.r.t. reduced-cost arc length */

  vector<double> dual_cost_double;
  dual_cost_double.resize(G->nb_nodes);
  for(i=0;i<G->nb_nodes;i++){
    dual_cost_double[i]=dual_cost[i];
    //    cout<<"mu["<<i<<"]="<<dual_cost_double[i]<<","<<dual_cost[i]<<")  ";
  }
  //cout<<endl;

      
  list<int> stable;
  double cplex_objvalue;
  
  A_cplex.set_objective_coefficient(dual_cost_double);


  bool stable_found = A_cplex.find_stableset(stable,cplex_objvalue);


  SCIP_Real reduced_cost = 1-cplex_objvalue;
	
  /* add stable set variable */
  if ( stable_found && SCIPisNegative(scip, reduced_cost) ){
    
	    list<int>::const_iterator it;

	    #ifdef OUTPUT_PRICER
	    cout<<"Stable found with reduced cost = "<<reduced_cost<<" : ";
	    for (it = stable.begin(); it != stable.end(); it++)
	      cout<<*it<<" ";
	    cout<<endl;
	    #endif
  
	    char var_name[255];
	    SCIPsnprintf(var_name, 255, "V_%d",M->L_var.size());
	    SCIPdebugMsg(scip, "new variable <%s>\n", var_name);

	    /* create the new variable: Use upper bound of infinity such that we do not have to care about
	     * the reduced costs of the variable in the pricing. The upper bound of 1 is implicitly satisfied
	     * due to the set partitioning constraints.
	     */
	    C_master_var *var = new C_master_var;
	    SCIPcreateVar(scip, &(var->ptr), var_name,
			  0.0,                     // lower bound
			  SCIPinfinity(scip),      // upper bound
			  1.0,                     // objective
			  SCIP_VARTYPE_INTEGER, // variable type
			  false, false, NULL, NULL, NULL, NULL, NULL);

	    /* add new variable to the list of variables to price into LP (score: leave 1 here) */
	    SCIPaddPricedVar(scip, var->ptr, 1.0);

	    /* add coefficient into the set partition constraints */
	    var->L_nodes.clear();
	    for (it = stable.begin(); it != stable.end(); it++){
	      SCIPaddCoefLinear(scip, M->V_node_ineq[*it], var->ptr, 1.0);
	      var->L_nodes.push_back(*it);
	    }

	    M->L_var.push_back(var);
     

	  }

#ifdef OUTPUT_PRICER
  SCIPwriteTransProblem(scip, "coloring.lp", "lp", FALSE);
  cout<<"************END PRICER******************"<<endl;
#endif


}


