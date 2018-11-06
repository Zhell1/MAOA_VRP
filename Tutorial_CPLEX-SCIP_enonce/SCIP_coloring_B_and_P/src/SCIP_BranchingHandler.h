#ifndef BranchingHandlerH
#define BranchingHandlerH

#include "scip/scip.h" 
#include "scip/cons_linear.h"

#include <scip/scipdefplugins.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"

#include "SCIP_master.h"
#include "SCIP_pricer.h"
#include "Graph.h"


using namespace std;


#define SCIP_DEBUG

// Current data of a node in the search tree
struct SCIP_ConsData {
  
  C_master_var *var;
  
  int rule; // indicates the rule condition
  
  struct SCIP_ConsData* father_data; // Pointer on the data of the father node

};




/*****
 * Create an "artificial" constraint that will be a included localy in the node for containg the node data
 *****/
void create_CteBranch(SCIP* scip_,  C_master_var * var, int rule, SCIP_ConsData *father_data , SCIP_CONS** cons);


///////////////////////////:::

class BranchingHandler : public scip::ObjConshdlr {

 public :
  
  C_Graph *G;
  ObjPricerColoring *pricer_ptr;
  
 BranchingHandler(SCIP* scip,  C_Graph *GG, ObjPricerColoring *ppricer_ptr) :
          scip::ObjConshdlr(
		scip,
		"BranchingHandler",                    // const char *  	name,
		"Handler For Branching Constraints",   // const char *  	desc,
		2000000, -2000000, -2000000,           // int sepapriority, int enfopriority, int checkpriority, 
		1, -1, 1, 0,                           // int sepafreq, int propfreq, int eagerfreq, int maxprerounds,	       
		FALSE, FALSE, FALSE,                   // delaysepa, delayprop, needscons,
		SCIP_PROPTIMING_BEFORELP,              // SCIP_PROPTIMING  	proptiming,
		SCIP_PRESOLTIMING_FAST                 // SCIP_PRESOLTIMING  	presoltiming 
	) 	

    { G = GG;
      pricer_ptr=ppricer_ptr;
    }

  
  /**
     *  Activation d'un noeud (appelée à chaque fois qu'on rentre dans un noeud)
     **/
    virtual SCIP_RETCODE scip_active(
	SCIP * 	scip,
	SCIP_CONSHDLR * conshdlr,
	SCIP_CONS * cons 
        );

    /**
     * Désactivation d'un noeud (appelée à chaque fois qu'on quitte un noeud
     * sauf si on le quitte pour aller dans un noeud fils)
     **/
    virtual SCIP_RETCODE scip_deactive(
	SCIP * 	scip,
	SCIP_CONSHDLR * 	conshdlr,
	SCIP_CONS * 	cons 
	);



      //////////////////////////:
      //////////////////////////
      
       /** transforms constraint data into data belonging to the transformed problem */
    virtual SCIP_RETCODE scip_trans(
	SCIP*              scip,               //**< SCIP data structure *
	SCIP_CONSHDLR*     conshdlr,           //**< the constraint handler itself *
	SCIP_CONS*         sourcecons,         //**< source constraint to transform *
	SCIP_CONS**        targetcons          //**< pointer to store created target constraint *
	);


    virtual SCIP_RETCODE scip_check(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	SCIP_SOL*          sol,                /**< the solution to check feasibility for */
	SCIP_Bool          checkintegrality,   /**< has integrality to be checked? */
	SCIP_Bool          checklprows,        /**< have current LP rows to be checked? */
	SCIP_Bool          printreason,        /**< should the reason for the violation be printed? */
	SCIP_Bool          completely,         /**< should all violations be checked? */
	SCIP_RESULT*       result              /**< pointer to store the result of the feasibility checking call */
	);
	
    virtual SCIP_RETCODE scip_enfolp(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	int                nusefulconss,       /**< number of useful (non-obsolete) constraints to process */
	SCIP_Bool          solinfeasible,      /**< was the solution already declared infeasible by a constraint handler? */
	SCIP_RESULT*       result              /**< pointer to store the result of the enforcing call */
	);
	
    virtual SCIP_RETCODE scip_enfops(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	int                nusefulconss,       /**< number of useful (non-obsolete) constraints to process */
	SCIP_Bool          solinfeasible,      /**< was the solution already declared infeasible by a constraint handler? */
	SCIP_Bool          objinfeasible,      /**< is the solution infeasible anyway due to violating lower objective bound? */
	SCIP_RESULT*       result              /**< pointer to store the result of the enforcing call */
	);
    
    virtual SCIP_RETCODE scip_lock(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS*         cons,               /**< the constraint that should lock rounding of its variables, or NULL if the
						*   constraint handler does not need constraints */
	int                nlockspos,          /**< no. of times, the roundings should be locked for the constraint */
	int                nlocksneg           /**< no. of times, the roundings should be locked for the constraint's negation */
	);
    
    virtual SCIP_RETCODE scip_sepalp(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	int                nusefulconss,       /**< number of useful (non-obsolete) constraints to process */
	SCIP_RESULT*       result              /**< pointer to store the result of the separation call */
	);
    
    virtual SCIP_RETCODE scip_sepasol(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	int                nusefulconss,       /**< number of useful (non-obsolete) constraints to process */
	SCIP_SOL*          sol,                /**< primal solution that should be separated */
	SCIP_RESULT*       result              /**< pointer to store the result of the separation call */
	);
};




#endif
