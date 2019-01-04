#ifndef SOLVE_RELAXED_PLNE_H
#define SOLVE_RELAXED_PLNE_H

#include <stdio.h>      /* printf */
#include <math.h>       /* sqrt */

#include <iostream>
#include <iomanip>
#include <string.h>
#include <sstream>

#include <iostream>
#include <fstream>

#include <unordered_map>
using namespace std;

#include "../../Graph/Graph.h"

#define epsilon 0.00001


///////////////////////////////
/////////////////////////////// NOTES 

// MIP formulation for the BinPacking problem
// Given a graph G=(V,E)

//   https://fr.wikipedia.org/wiki/Probl%C3%A8me_de_bin_packing

/*
Approche choisie:
- relaxer la contrainte du “m tourn´ees” en r´esolvant (par PLNE) un probl`eme avec un nombre potentiellement
plus grand de tourn´ees; puis en confiant l’objectif de r´eduire le nombre de tourn´ees
`a une ´etape ult´erieure.
*/

///////////////////////////////
///////////////////////////////

//renvoie en return le nombre de boites utilisées, modifie le vecteur solution pour mettre la solution dedans
//bool activateoutput = true; //active this to print outputs & visualisation to pdf file
int solve_relaxedPLNE(C_Graph* G, string filename, vector<int> *solution_vec_out, bool activateprint, bool activateoutput) {

  string name,nameext,nameextsol;
  int i,j;
  list<int>::const_iterator it;

  name=filename;
  nameext=name+".dim";
  nameextsol=name+".color";

  //////////////
  //////  CPLEX INITIALIZATION
  //////////////

  IloEnv   env;
  IloModel model(env);

  ////////////////////////
  //////  VAR
  ////////////////////////

  //N borne sup sur le nombre de boites à utiliser (au pire on met chaque objet dans sa propre boite)
  int N = (*G).nb_nodes; // N = |V|
  //y
  vector<IloNumVar> y;
  y.resize(N); 

  ///////     
  ///////     IMPORTANT : ALL LOOPS START AT 1 BECAUSE THE FIRST NODE IS THE DEPOT AND WE DON'T WANT IT HERE
  ///////     

  //y_j var binaire {0,1}, égale à 1 si la boîte  j est utilisée, 0 sinon.
  for(j = 1; j < N; j++) {
    y[j] = IloNumVar(env, 0.0, 1.0, ILOINT);
    ostringstream varname;
    varname.str("");
    varname<<"y_"<<j;
    y[j].setName(varname.str().c_str());
  }

  //x_i_j € {0,1}
  vector<vector<IloNumVar> > x;
  x.resize(N); 

  for (i=1;i< N;i++){
    vector<IloNumVar> x_i;
    x_i.resize(N); 

    for (j=1;j< N;j++) {
      x_i[j] = IloNumVar(env, 0.0, 1.0, ILOINT); 
      ostringstream varname;
      varname.str("");
      varname<<"x_"<<i<<"_"<<j;
      x_i[j].setName(varname.str().c_str());
    }
    x[i] = x_i;
  }

  //////////////
  //////  CST
  //////////////

  IloRangeArray CC(env);
  int nbcst=0;

  // première contrainte:
  //      
  //         n
  //      somme (x_i_j * c_i) <= C * y_j         pour tout j = 1 à n
  //       i = 1
  //      
  //      Signifie qu'on ne peut dépasser la taille d'une boîte pour un rangement.
  //      La partie droite de l'inégalité oblige y_j à prendre la valeur 1
  //          dès qu'un article est rangé dans la boîte j

  int C_const = (*G).VRP_capacity;
  int c_i;

  //for every j
  for (j = 1; j < N; j++){
    IloExpr cst(env);
    //pour tout i
    for(i = 1; i < N; i++) {
      c_i = (*G).get_node_by_id_startat0(i)->VRP_demand;
      cst += x[i][j] * c_i;
    }
    cst += - C_const * y[j];
    CC.add(cst<=0);
    ostringstream cstname;
    cstname.str("");
    cstname<<"Cst_firstype_"<<j ;//<< "  " << cst;
    if(activateprint) cout << cstname.str().c_str() << endl;
    CC[nbcst].setName(cstname.str().c_str());
    nbcst++;
  }

  // deuxième contrainte:
  //      
  //         n
  //      somme (x_i_j) = 1         pour tout i = 1 à n
  //       j = 1
  //      
  //      impose à tous les objets d'être rangés dans une boîte et une seule. 

  //for every i
  for (i = 1; i < N; i++){
    IloExpr cst(env);
    for (j = 1; j < N; j++){
      cst+=x[i][j];
    }
    CC.add(cst==1);
    ostringstream cstname;
    cstname.str("");
    cstname<<"Cst_secondtype_"<<i;
    if(activateprint) cout << cstname.str().c_str() << endl;
    CC[nbcst].setName(cstname.str().c_str());
    nbcst++;
  }

  model.add(CC);

  //////////////
  ////// OBJ
  //////////////
  
  IloObjective obj=IloAdd(model, IloMinimize(env, 0.0));
  
  //minimize   sum_for_j=1_to_n  (y_j)
  for (j = 1; j < N ; j++){
    obj.setLinearCoef(y[j],1);
  }

  ///////////
  //// RESOLUTION
  //////////

  IloCplex cplex(model);

  if(activateprint==false) cplex.setOut(env.getNullStream());

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

  string sortielp = filename+"sortie.lp";
  cplex.exportModel(sortielp.c_str());


  if ( !cplex.solve() ) {
    env.error() << "Failed to optimize LP" << endl;
    exit(1);
  }
 
  env.out() << "Solution status = " << cplex.getStatus() << endl;
  env.out() << "Solution value  = " << cplex.getObjValue() << endl;

  ///////
  /////// retrieving solution
  ////////////////:

  //on affiche les valeurs des boites (1=utilisé, 0=inutile)
  //for(int j = 1; j < N; j++)
  //  cout << j<< " -> " << cplex.getValue(y[j]) << endl;

  vector<int>   sol_x_i;
  vector<int> unique_values = {};
  sol_x_i.resize(N);
  for(i = 1; i < N; i++) {
    int val = 0;
    for(j=1; j < N; j++) {
      if(cplex.getValue(x[i][j]) == 1) {
        val = j; //article i rangé dans la boite j
        if(activateprint) cout << "article_" << i <<"\t\tin box : " << j << endl;
        solution_vec_out->push_back(j);
        unique_values.push_back(j);
        break;
      }
    }
    sol_x_i[i] = val;
  }
  // sort and remove duplicate elements=
  std::sort(unique_values.begin(), unique_values.end()); // 1 1 2 2 3 3 3 4 4 5 5 6 7 
  auto last = std::unique(unique_values.begin(), unique_values.end());
  // v now holds {1 2 3 4 5 6 7 x x x x x x}, where 'x' is indeterminate
  unique_values.erase(last, unique_values.end()); 
  cout << "unique_values : " ;
  for (int i : unique_values)
      std::cout << i << " ";
  std::cout << "\n";
  //on génère une map qui associe à chaque valeur sa position à partir de 1
  unordered_map<int, int> map_values;
  int z = 1;
  cout << "map_values : " ;
  for (int i : unique_values) {
     map_values.insert(std::make_pair(i, z));
     cout << "("<<i<<"->"<<z<<") " ;
     z++;
  }
  cout << endl;
  //maintenant on va post-traité le vecteur de solution pour être sûr de n'avoir que des valeurs 1,2,3,4 etc
  //car on peut avoir des valeurs du style: 1,2,287,548.. surtout avec de grands graphes
  solution_vec_out->clear();
  for(i = 1; i < N; i++) {
    sol_x_i[i] = map_values.at(sol_x_i[i]);
    //cout << "sol_x_i["<<i<<"] = "<<sol_x_i[i]<< endl;;
    solution_vec_out->push_back(sol_x_i[i]);
  }
  //cout << endl;

  // FIN du traitement de sol_x_i

  int sol_nb_box= 0;
  for(j = 1; j < N; j++) {
    if(cplex.getValue(y[j]) == 1) {
      sol_nb_box++; //y_j égale à 1 si la boite j est utilisée
    }
  }
  if(activateprint) cout << sol_nb_box << " BOXES USED " << endl;

  //////////////
  //////  CPLEX's ENDING
  //////////////
  env.end();

  //////////////
  //////  OUTPUT
  //////////////

  if(activateoutput) {

    ofstream ficsol(nameextsol.c_str());
    
    //pour chaque noeud on écrit la boite (tournée) à laquelle il appartient
    for(i = 1; i < N; i++) 
      ficsol<<sol_x_i[i]<<" ";

    ficsol.close();

    cout << "wrote solution to file: " << nameextsol.c_str() << endl;

    //////////////
    ////// second output using the viewerColor
    //////////////

    // NOW we write the output files as .color and .pdf

    // save to .color and reload it from the file
    string nameext2=name+".color";

    ifstream fic2(nameext2.c_str());

    if (fic2.fail()){
      cerr<<"file "<<nameext2<<" not found"<<endl;
      return 1;
    }

    vector<int> sol;
    sol.resize(N);
    
    for (i=1; i < N; i++)
      fic2>>sol[i];
    
    fic2.close();

    // create PDF with solution
    //G->write_dot_G_color(name.c_str(),sol);
    //cout << "wrote visualisation of solution to file: " << name.c_str() << "_G_color.pdf" << endl;

    //create svg with solution
    G->write_dot_G_color_svg(name.c_str(),sol);

  } // end of :  if(activateoutput) {

  // return number of boxes used
  return sol_nb_box ;
}


#endif