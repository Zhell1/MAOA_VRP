#ifndef SOLVE_MTZ_PLNE_H
#define SOLVE_MTZ_PLNE_H

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

//BACKUP, DONT DELETE THIS BUT CREATE ANOTHER FUNCTION SO THAT WE CAN COMPARE THE PERFORMANCES LATER!

//PLNE utilisant la formulation compacte MTZ
void optimizeMTZ_withsymmetry(vector<vector<int>> *tournees, C_Graph* G, string filename){
	//m est le nombre de tournées
	int m = tournees->size();
	int Q = (*G).VRP_capacity; // capacité max des véhicules
	int i,j,c;

  string name=filename;
	
	IloEnv   env;
	IloModel model(env);

  cout << endl <<  "\n*** starting MTZ optimization ***\n"<< endl;

	//TODO trouver un moyen de casser la symetrie

	////////////////////////
	//////  VAR
	////////////////////////

	//DECISION
	//xij vaut 1 si l'arete (i,j) est dans une des tournees
	//wic charge restante dans le camion c au sommet i
	
	//note : pour minimiser le nombre de tournees on peut rajouter une variable tc qui vaut 1 si la tournee est non vide
	
	//DONNEES
	//cij cout de l'arete
	//Q charge max d'un vehicule
	//di demande du sommet i

	//x_i_j
	vector<vector<IloNumVar>> x;
	x.resize(G->nb_nodes);

	for (i=0; i<G->nb_nodes; i++){
    x[i].resize(G->nb_nodes);
		for (j=0; j<G->nb_nodes; j++) {
			if (i!=j){ // thomas: ne pas faire i < j ici, on veut bien créer les variables
				x[i][j]=IloNumVar(env, 0.0, 1.0, ILOINT);
				ostringstream varname;
				varname.str("");
				varname<<"x_"<<i<<"_"<<j;
        cout << varname.str() << endl;
				x[i][j].setName(varname.str().c_str());
			}
		}
	}

  // on appelle le sommet 0 le dépôt et les sommets NC = N \ {0} les revendeurs (ou clients).
	//w_i
	vector<IloNumVar> w;
	w.resize(G->nb_nodes); // prends pas en compte le sommet 0

  // thomas: tu avais fais des w_i_c mais en fait ce sont juste des w_i et on ne doit pas prendre le sommet 0

	for (i=1; i < G->nb_nodes; i++){
      w[i] = IloNumVar(env, 0.0, Q, ILOFLOAT); // 0 ≤ wi ≤ Q ∀i ∈ NC
			ostringstream varname;
			varname.str("");
			varname<<"w_"<<i;
      cout << varname.str() << endl;
			w[i].setName(varname.str().c_str());
	}

	//////////////
	//////  CST
	//////////////

	IloRangeArray CC(env);
	int nbcst=0;
	
	//TODO : tenter de couper la matrice des contraintes en 2 en remplacant i!=j par i < j et en ajoutant x[i,j] = x[j,i] en contrainte 
	// OU mettre a jour toutes les autres contraintes



	// la somme de toutes les aretes partant ou arrivant de j fait 1 (1 partant + 1 arrivant)

  //nombre max d'entrees/sorties depuis le depot <= m
  IloExpr c3(env);
  for (j=1; j < G->nb_nodes; j++){  //ok verifié
    //c3+=x[0][j]; // ok car on a tjr 0 < j
    c3 += x[0][j] ;
  }
  CC.add(c3<=m);
  ostringstream nomcst;
  nomcst.str("");
  nomcst<<"CstMax_from_depot";
  cout << nomcst.str() << endl;
  CC[nbcst].setName(nomcst.str().c_str());
  nbcst++;
  
  
  IloExpr c4(env);
  for (i=1; i < G->nb_nodes; i++){   //ok verifié
    c4 += x[i][0];
  }
  CC.add(c4<=m);
  ostringstream nomcst2;
  nomcst2.str("");
  nomcst2<<"CstMax_to_depot";
  cout << nomcst2.str() << endl;
  CC[nbcst].setName(nomcst2.str().c_str());
  nbcst++;
  

  for (i=1; i < G->nb_nodes; i++){ // ∀i ∈ NC
    IloExpr c2(env);
    for (j=0; j < G->nb_nodes; j++){
      if(i!=j) {
        c2+=x[i][j];
      }
    }
    CC.add(c2==1);
    ostringstream nomcst;
    nomcst.str("");
    nomcst<<"Cst_from_"<<i;
    cout << nomcst.str() << endl;
    CC[nbcst].setName(nomcst.str().c_str());
    nbcst++;
  }

	for (j=1; j < G->nb_nodes; j++){ // ∀i ∈ NC
		IloExpr c1(env);
		for (i=0; i < G->nb_nodes; i++){
			if (i != j){
         c1+=x[i][j];
       }
		}
		CC.add(c1==1);
		ostringstream nomcst;
		nomcst.str("");
		nomcst<<"Cst_to_"<<j;
    cout << nomcst.str() << endl;
		CC[nbcst].setName(nomcst.str().c_str());
		nbcst++;
	}
	
	
	// contraintes MTZ (pour i quelconque et j!=0)
	for (i=1; i < G->nb_nodes; i++){

		int di = G->V_nodes[i].VRP_demand;

		for (j=0; j < G->nb_nodes; j++){
  		if (i!=j)
      {
  				IloExpr c5(env);
          if(j!=0)
    			  c5 += w[i] - w[j] - di +  (Q + di ) * ( 1 - x[i][j] ); 
          else
            c5 += w[i] - 0    - di +  (Q + di ) * ( 1 - x[i][j] ); 

          cout << "w_"<<i<<" - w_"<<j<<" - "<<di << " + ("<<Q+di<<" * (1-xij) ) >= 0"<<endl;

  				CC.add(c5>=0);
  				ostringstream nomcst3;
  				nomcst3.str("");
  				nomcst3<<"Cst_MTZ_"<<i<<"_"<<j;
  				CC[nbcst].setName(nomcst3.str().c_str());
  				nbcst++;
			}
		}
	}



	model.add(CC);


	//////////////
	////// OBj
	//////////////

  // min sum( cij * xij ) pour tout (i,j)€A 

	IloObjective obj=IloAdd(model, IloMinimize(env, 0.0));

  //ici on prend bien en compte les arêtes partants et arrivants au sommet source 0
	for (i=0; i < G->nb_nodes; i++){
		for (j=0; j < G->nb_nodes; j++){
			if (i!=j) {// previously if(i!=j) 
        obj.setLinearCoef(x[i][j], G->lengthTSP(i,j));
        //obj.setLinearCoef(x[i][j], 0); // affiche la première solution trouvée valide
    }
   }
  }
			
			
	///////////
	//// RESOLUTION
	//////////

	
	cout << CC << endl;
	
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


	string sortielp = /*filename+*/"sortie.lp";
	cplex.exportModel(sortielp.c_str());

	if ( !cplex.solve() ) {
		env.error() << "Failed to optimize LP" << endl;
		exit(1);
	}


	env.out() << "Solution status = " << cplex.getStatus() << endl;
	env.out() << "Solution value  = " << cplex.getObjValue() << endl;
	

  //affichage des w_i
  /*cout<<"---"<<endl;
  for(int i = 1; i < G->nb_nodes; i++) {
    cout <<"w_" << i<<" = " << cplex.getValue(w[i]) << endl;
  }
  cout<<"---"<<endl;
  */

  // on enregistre tous les arcs du graphe dans sol
  // et on va aussi exporter la solution dans le pointeur d'entrée "tournees"
  cout << "\n MTZ arcs in graph solution : " << endl;
	list<pair<int,int>> sol; // marche pour tous
  map<int, int> solmap; // marche pour tous sauf le depot (car il y a plusieurs entrées avec le mm depot)
	for(i=0; i < G->nb_nodes; i++) {
		for (j=0; j < G->nb_nodes;j++) {
			if (i!=j && cplex.getValue(x[i][j])>1-epsilon) {
				sol.push_back(make_pair(i,j));
        if(i != 0) solmap.insert(make_pair(i,j));
        cout << "arc ("<<i<<","<<j<<")" << endl;
      }
    }
  }
  //maintenant on sauvegarde ça dans "tournees"
  bool display_debug_mtz = false;
  if(display_debug_mtz)  cout << "solution MTZ par tournées : "<< endl;
  int nbtournee =0;
  tournees->clear();
  //parcourir toutes les tournées depuis le dépot
  for(auto it = sol.begin(); it != sol.end(); ++it) {
    if(it->first == 0) {
      vector<int> tempvecint;
      //quand on trouve une tournée passant par 0 on la parcours jusqu'à revenir à 0
      int courant = it->second;
      if(display_debug_mtz) cout << "tournée : "<< it->first;
      while(courant != 0) {
        tempvecint.push_back(courant);
          if(display_debug_mtz) cout << "\t -> " << courant << "\td="<< G->get_node_by_id_startat0(courant)->VRP_demand << "\tw="<< cplex.getValue(w[courant]) << endl;
          courant = solmap.find(courant)->second;
      }
      if(display_debug_mtz) cout << "\t -> 0"<< endl;
      tournees->push_back(tempvecint);
      nbtournee++;
    }
  }


  //END CPLEX
	env.end();

}


#endif