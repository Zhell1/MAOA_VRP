#ifndef __SCIP_PRICER_COLORING_H__
#define __SCIP_PRICER_COLORING_H__

#include "objscip/objscip.h"
#include "scip/pub_var.h"

#include <vector>
#include <list>

#include "Graph.h"
#include "SCIP_master.h"
#include "Find_Stable_set_with_Cplex.h"

using namespace std;
using namespace scip;


/** pricer class */
class ObjPricerColoring : public ObjPricer{
public:

   C_Graph *G;

   C_master_coloring *M;

   Cplex_pricing_algo A_cplex;
  

   /** Constructs the pricer object with the data needed */
   ObjPricerColoring(
      SCIP*                               scip,        /**< SCIP pointer */
      const char*                         p_name,      /**< name of pricer */
      C_Graph*                      G,
      C_master_coloring *           M 
      );

   /** Destructs the pricer object. */
   virtual ~ObjPricerColoring();

   /** initialization method of variable pricer (called after problem was transformed) */
   virtual SCIP_DECL_PRICERINIT(scip_init);

   /** reduced cost pricing method of variable pricer for feasible LPs */
   virtual SCIP_DECL_PRICERREDCOST(scip_redcost);

   /** perform pricing */
   void coloring_pricing(
      SCIP*              scip               /**< SCIP data structure */
      );


   
};

#endif
