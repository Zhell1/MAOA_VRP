#ifndef __SCIP_MASTER_H__
#define __SCIP_MASTER_H__


#include <vector>
#include <list>

/* scip includes */
#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

using namespace std;
using namespace scip;


class C_master_var{
 public:
  
   // Keep a pointer on every variables of the Master program
  SCIP_VAR * ptr;

  // Keep the list of nodes of the stable set corresponding to the variable
  list<int> L_nodes;
    
};



class C_master_coloring{
 public:


  // Keep a pointer on every constraint of the Master program
  vector<SCIP_CONS*> V_node_ineq;

  // Keep informations on every variables of the Master program
  list<C_master_var*> L_var;

};


#endif
