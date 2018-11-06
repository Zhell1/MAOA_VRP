#include"SCIP_BranchingHandler.h"


//#define OUTPUT_HANDLER

//////////////////////////////////////////////
//////////////////////////////////////////////
void create_CteBranch(SCIP* scip, C_master_var* var, int rule,  SCIP_ConsData *father_data, SCIP_CONS** cons) {


#ifdef OUTPUT_HANDLER
   cout << " ------ CREATE A CONSTRAINT ASSOCIATED TO A NODE   ---------------  \n";
#endif


    // initialise les donnees specifiques au noeud fils
    SCIP_ConsData* consdata = new SCIP_ConsData;
    SCIP_CONSHDLR* conshdlr = SCIPfindConshdlr(scip, "BranchingHandler");
    
#ifdef OUTPUT_HANDLER
        if (conshdlr==NULL) cout<<"CONSTRAINT HANDLER NOT FOUND -> CHECK SCIP_DEBUG TO SEE ITS PARAMETERS"<<endl;
#endif
	
    // cree la constrainte de branchement du noeud fils
    consdata->var = var;
    consdata->rule=rule;
    consdata->father_data = father_data;
//??????    consdata->rule = _rule;

 
    SCIPcreateCons(scip, cons, "BranchingCste", conshdlr, consdata,
    							FALSE, //initial
    							FALSE, //separate
    							FALSE, //enforce
    							FALSE, //check
    							TRUE,  //propagate
    							TRUE,  //local
    							FALSE, //modifiable
    							FALSE, //dynamic
    							FALSE, //removable
		                                        TRUE); //stickinganode

    
    
#ifdef OUTPUT_HANDLER
   cout << " ------ END CREATION  ---------------  \n";
#endif
 
}


//////////////////////////////////////////////
//////////////////////////////////////////////
SCIP_RETCODE BranchingHandler::scip_active(SCIP * scip, SCIP_CONSHDLR * conshdlr, SCIP_CONS * cons) {

#ifdef OUTPUT_HANDLER
    cout << " --------------------- Active handler ---------------  \n";
#endif
 
    SCIP_ConsData *consdata = SCIPconsGetData(cons);
    
  #ifdef OUTPUT_HANDLER
  cout<<"Treated  "<<cons->name<<" : ";
  list<int>::iterator it;
  for (it=consdata->var->L_nodes.begin();it!=consdata->var->L_nodes.end();it++)
    cout<<*it<<" ";
  cout<<" with rule "<<consdata->rule<<endl;
  #endif


 

  if (consdata->rule==0)
    SCIPchgVarUbNode(scip, SCIPgetCurrentNode(scip), consdata->var->ptr , 0.0);
          // changes upper bound of variable in the given node; if possible, adjust bound
          //  to integral value; doesn't store any inference information in the bound change,
          //such that in conflict analysis, this change is treated like a branching decision

    if (consdata->rule==1)
      SCIPchgVarLbNode(scip, SCIPgetCurrentNode(scip), consdata->var->ptr , 1.0);
    
    #ifdef OUTPUT_HANDLER
      cout<<"Var ";
      for (it=consdata->var->L_nodes.begin();it!=consdata->var->L_nodes.end();it++)
	cout<<*it<<" ";
      cout<<" set to "<<consdata->rule<<endl;
    #endif
      //      
      //    curr=curr->father_data;
      //  }


      #ifdef OUTPUT_HANDLER
      cout<<"List of forbidden stable sets:"<<endl;
      #endif
      
      list<list<int> > *LL_notequal=new list<list<int> >;
      LL_notequal->clear();
      SCIP_ConsData *curr=consdata;
      while (curr!=NULL){
	LL_notequal->push_back(curr->var->L_nodes);
	#ifdef OUTPUT_HANDLER
	  for (it=curr->var->L_nodes.begin();it!=curr->var->L_nodes.end();it++)
	    cout<<*it<<" ";
	    cout<<endl;
	#endif
	curr=curr->father_data;
      }
	#ifdef OUTPUT_HANDLER
      cout<<"End the List of forbidden stable sets."<<endl;
      #endif

      pricer_ptr->A_cplex.set_forbidden_stable_set(LL_notequal);
  

#ifdef OUTPUT_HANDLER
    cout << " --------------------- Fin Active handler ---------------  \n";
#endif


    return SCIP_OKAY;
}

//////////////////////////////////////////////
//////////////////////////////////////////////
SCIP_RETCODE BranchingHandler::scip_deactive(SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_CONS*cons){
#ifdef OUTPUT_HANDLER
	cout << " --------------------- Desactive handler ---------------  \n";
#endif

	assert(conshdlr != NULL);
	assert(cons != NULL);

	delete pricer_ptr->A_cplex.L_notequal;
	pricer_ptr->A_cplex.L_notequal=NULL;
	
	return SCIP_OKAY;
    }


//////////////////////////////////////////////
/** transforms constraint data into data belonging to the transformed problem */
    SCIP_RETCODE BranchingHandler::scip_trans(
	SCIP*              scip,               //**< SCIP data structure *
	SCIP_CONSHDLR*     conshdlr,           //**< the constraint handler itself *
	SCIP_CONS*         sourcecons,         //**< source constraint to transform *
	SCIP_CONS**        targetcons          //**< pointer to store created target constraint *
	) {

#ifdef OUTPUT_HANDLER
	std::cout << " --------------------- Trans handler ---------------  \n";
#endif
	
   SCIP_CONSDATA* sourcedata;
   SCIP_CONSDATA* targetdata;

   sourcedata = SCIPconsGetData(sourcecons);
   targetdata = NULL;

   targetdata= new SCIP_CONSDATA;
   targetdata->var = sourcedata->var;
   targetdata->father_data = sourcedata->father_data;

   SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, targetdata,
				  SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
				  SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons),
				  SCIPconsIsLocal(sourcecons), SCIPconsIsModifiable(sourcecons), 
				  SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons), SCIPconsIsStickingAtNode(sourcecons));


    
	return SCIP_OKAY;
    }


/////////////////////////////////////////////
    SCIP_RETCODE BranchingHandler::scip_check(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	SCIP_SOL*          sol,                /**< the solution to check feasibility for */
	SCIP_Bool          checkintegrality,   /**< has integrality to be checked? */
	SCIP_Bool          checklprows,        /**< have current LP rows to be checked? */
	SCIP_Bool          printreason,        /**< should the reason for the violation be printed? */
	SCIP_Bool          completely,         /**< should all violations be checked? */
	SCIP_RESULT*       result) {

#ifdef OUTPUT_HANDLER
	std::cout << " --------------------- Check handler ---------------  \n";
#endif


	*result = SCIP_FEASIBLE;
	return SCIP_OKAY;

    }
	
    SCIP_RETCODE BranchingHandler::scip_enfolp(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	int                nusefulconss,       /**< number of useful (non-obsolete) constraints to process */
	SCIP_Bool          solinfeasible,      /**< was the solution already declared infeasible by a constraint handler? */
	SCIP_RESULT*       result) {

#ifdef OUTPUT_HANDLER
	std::cout << " --------------------- Enfolp handler ---------------  \n";
#endif


	*result = SCIP_FEASIBLE;
	return SCIP_OKAY;
    }
	
    SCIP_RETCODE BranchingHandler::scip_enfops(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	int                nusefulconss,       /**< number of useful (non-obsolete) constraints to process */
	SCIP_Bool          solinfeasible,      /**< was the solution already declared infeasible by a constraint handler? */
	SCIP_Bool          objinfeasible,      /**< is the solution infeasible anyway due to violating lower objective bound? */
	SCIP_RESULT*       result) {

#ifdef OUTPUT_HANDLER
	std::cout << " --------------------- Enfops handler ---------------  \n";
#endif


	*result = SCIP_FEASIBLE;
	return SCIP_OKAY;
    }

    SCIP_RETCODE BranchingHandler::scip_lock(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS*         cons,               /**< the constraint that should lock rounding of its variables, or NULL if the
						*   constraint handler does not need constraints */
	int                nlockspos,          /**< no. of times, the roundings should be locked for the constraint */
	int                nlocksneg) {

#ifdef OUTPUT_HANDLER
	std::cout << " --------------------- Lock handler ---------------  \n";
#endif


	return SCIP_OKAY;
    }
		
    SCIP_RETCODE BranchingHandler::scip_sepalp(
	SCIP*              scip,               /**< SCIP data structure */
	SCIP_CONSHDLR*     conshdlr,           /**< the constraint handler itself */
	SCIP_CONS**        conss,              /**< array of constraints to process */
	int                nconss,             /**< number of constraints to process */
	int                nusefulconss,       /**< number of useful (non-obsolete) constraints to process */
	SCIP_RESULT*       result) {

#ifdef OUTPUT_HANDLER
	std::cout << " --------------------- Sepalp handler ---------------  \n";
#endif


	*result = SCIP_DIDNOTRUN;
	return SCIP_OKAY;
    }

    SCIP_RETCODE BranchingHandler::scip_sepasol(SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_CONS** conss, 
						int nconss, int nusefulconss, SCIP_SOL* sol, SCIP_RESULT* result){

#ifdef OUTPUT_HANDLER
	std::cout << " --------------------- Sepasol handler ---------------  \n";
#endif

	*result = SCIP_DIDNOTRUN;
	return SCIP_OKAY;
    }
