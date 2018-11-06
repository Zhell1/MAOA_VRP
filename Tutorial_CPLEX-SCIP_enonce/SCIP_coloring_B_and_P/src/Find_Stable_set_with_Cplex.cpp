#include "Find_Stable_set_with_Cplex.h"

using namespace std;

#define eps 1e-6

#define CPLEX_OUTPUT


bool test_equal(list<int>& l1, list<int>& l2){
  list<int>::const_iterator it1,it2;
  it1=l1.begin();
  it2=l2.begin();
  
  while ( (it1!=l1.end()) && (it2!=l2.end()) && (*it1==*it2) ){
    it1++;
    it2++;
  }

  if ( (it1!=l1.end()) || (it2!=l2.end()) )      
    return false;
  else
    return true;
}

bool test_include(list<int>& l, list<list<int> >* L){
  list<list<int> >::iterator it;

  if (L==NULL)
    return false;
  
  it=L->begin();
  while ( (it!=L->end()) && (!test_equal(l,*it)) )
    it++;

  if (it!=L->end())
    return true;    
  else
    return false;

}




ILOINCUMBENTCALLBACK1(MyCallback_NotEqual,
		      Cplex_pricing_algo *, CPX){
  int i;
  list<int> S;
  list<int>::const_iterator it;
  
  S.clear();
  for (i=0;i<CPX->G->nb_nodes;i++)
    if (getValue(CPX->x[i])>1-eps) S.push_back(i);

  #ifdef CPLEX_OUTPUT  
  cout<<"Incumbent solution : ";
  for (it=S.begin();it!=S.end();it++)
    cout<<*it<<" ";
  cout<<endl;
    if (getSolutionSource()==NodeSolution)
      cout<<"found as the solution to an LP-relaxation of a node in the search tree."<<endl;
    if (getSolutionSource()==HeuristicSolution)
       cout<<"found by a CPLEX internal heuristic."<<endl; 
  #endif


    if (test_include(S,CPX->L_notequal)){
      reject();      
      #ifdef CPLEX_OUTPUT
      cout<<"REJECT"<<endl;
      #endif
    }

}





void Cplex_pricing_algo::initialization(C_Graph *GG){

  G=GG;
  
  model= IloModel(env);


  ////////////////////////
  //////  VAR
  ////////////////////////

  int i,k;
  
  //x = new IloNumVarArray(*env, G->nb_nodes, 0.0, 1.0, ILOINT);
  x = IloNumVarArray(env, G->nb_nodes, 0.0, 1.0, ILOINT);
  for(i = 0; i < G->nb_nodes; i++) {
    ostringstream varname;
    IloNumVar v(env, 0.0, 1.0, ILOINT);
    varname.str("");
    varname<<"x_"<<i;
    x[i].setName(varname.str().c_str());

    
  }

  //////////////
  //////  EDGE INEQUALITIES
  //////////////

  IloRangeArray CC(env);
  int nbcst=0;

  // Cst x_i + x_j \le 1 for every edges ij in E

  for (k=0;k<G->nb_links;k++){
      IloExpr cst(env);
      cst+=x[G->V_links[k]->v1]+x[G->V_links[k]->v2];
      CC.add(cst<=1);
      ostringstream cstname;
      cstname.str("");
      cstname<<"CEdge_"<<G->V_links[k]->v1<<"_"<<G->V_links[k]->v2;
      CC[nbcst].setName(cstname.str().c_str());
      nbcst++;
  }
  
  
  model.add(CC);


  //////////////
  ////// OBJ
  //////////////


  // Initialization without any value
  obj = IloAdd(model, IloMaximize(env, 0.0));
  

   //////////////
  ////// CPLEX 
  //////////////

  
  cplex = IloCplex(model);

  cplex.use(MyCallback_NotEqual(env,this)); 


  set_forbidden_stable_set(NULL);

  #ifdef CPLEX_OUTPUT
  #else
     cplex.setOut(env.getNullStream());
     cplex.setWarning(env.getNullStream());
   #endif
}


void Cplex_pricing_algo::set_objective_coefficient(const vector<double>& obj_coeff){
 int i;

  for (i=0;i<G->nb_nodes;i++)
    obj.setLinearCoef(x[i],obj_coeff[i]);

}




void Cplex_pricing_algo::set_forbidden_stable_set(list<list<int> >* LL_notequal){
  L_notequal=LL_notequal;

}


void Cplex_pricing_algo::aff_forbidden_stable_set(){
 list<list<int> >::iterator it;
 list<int>::iterator iti;

 cout<<"Forbidden stable set list : ";
 if (L_notequal==NULL){
   cout<<"NULL"<<endl;
 }
 else{
   cout<<endl;
   for(it=L_notequal->begin(); it!=L_notequal->end(); it++){
     for (iti=it->begin();iti!=it->end();iti++)
       cout<<*iti<<" ";
     cout<<endl;
   }
   cout<<"end of list"<<endl;
}

}

// return true if a stable set have been found by Cplex
bool  Cplex_pricing_algo::find_stableset(list<int>& stable, double &objvalue) {
  list<int>::const_iterator it;
   int i;

  #ifdef CPLEX_OUTPUT
  cout<<endl<<" ************************* LAUCH PRICER with  CPLEX"<<endl<<endl;
  aff_forbidden_stable_set();
  #endif
 
	  
  #ifdef CPLEX_OUTPUT
  cplex.exportModel("sortie.lp");
  #endif
  
  if ( !cplex.solve() ) {
    env.error() << "Failed to optimize Pricer with Cplex" << endl;
    exit(1);
  }

  if (cplex.getStatus()==CPX_STAT_INFEASIBLE){
    #ifdef CPLEX_OUTPUT
    cout<<"NO SOLUTION"<<endl;
    cout<<endl<<" ************************* END PRICER with CPLEX"<<endl<<endl;
    #endif
    
    return false;
  }
  else{
    stable.clear();

    objvalue=cplex.getObjValue();
    
    for(i=0;i<G->nb_nodes;i++){
      if (cplex.getValue(x[i])>1-eps)
	stable.push_back(i);
    }


    #ifdef CPLEX_OUTPUT
    cout<<"Produced column : ";
    for (it=stable.begin();it!=stable.end();it++)
      cout<<*it<<" ";
    cout<<endl;
    cout<<"Objective value : "<<objvalue<<endl;
    #endif
  
    if (!test_include(stable, L_notequal)){
    #ifdef CPLEX_OUTPUT
      cout<<"NEW STABLE";
      cout<<endl<<" ************************* END PRICER with CPLEX"<<endl<<endl;     
      #endif
    return true;
    }
    else{
      #ifdef CPLEX_OUTPUT
      cout<<"ALREADY KNOWN -> REJECTED ";
      cout<<endl<<" ************************* END PRICER with CPLEX"<<endl<<endl;     
      #endif
      return false;
    }
  }
  

}


