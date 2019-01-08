#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include"../../Graph/Graph.h"
#include "./VRPfileParser.h"
#include "./solve_relaxedPLNE.h"

#define epsilon 0.00001

using namespace std;

string vectorint_tostring(vector<int> my_vector) {
  std::stringstream result;
  std::copy(my_vector.begin(), my_vector.end(), std::ostream_iterator<int>(result, " "));
  return result.str().c_str();
}

void print_all_tournees(vector<vector<int>> tournees, C_Graph* G) {
  int nb_box_used = tournees.size();
  //on affiche toutes les tournées et leurs couts:
  cout << "tournées : "<< endl;
  //pour chacune des tournées
  for(int i = 0; i < nb_box_used; i++) {
      cout << "\t tournée #"<<i<< " (cost= "<< G->get_route_cost(tournees.at(i)) << ") (demand="<<G->get_route_demand(tournees.at(i)) <<") : ";
      //on lit toute la tournée
      for(int j = 0; j < tournees.at(i).size(); j++){
          //on extrait les sommets de la tournée à enregistrer
          cout << tournees.at(i).at(j) <<" ";
      }
      cout << endl;
  }
  cout << "\t total_cost = " << G->get_VRP_cost(tournees) << endl;
}

void optimize_2opt_internalRoutes(vector<vector<int>> *tournees, C_Graph* G) {
   //voisinage 2opt pour optimiser chaque tournée indépendament
  cout << endl <<  "*** starting 2opt optimization of internal routes ***"<< endl;

  bool print_alldebug_2opt = false;
  bool print_shortdebug = true; //prints less stuff

  int nb_box_used = tournees->size();
  //on parcours toutes les tournées une par une : 
  for(int ibox = 0; ibox < nb_box_used; ibox++) {
    vector<int> *tournee = &(tournees->at(ibox)); // passe par un pointeur pour pouvoir modifier direct les valeurs
    //tournee->at(0) = 1000; cout << tournees.at(i).at(0) << " ???"<< endl; // test de modification OK
    float initial_cost = G->get_route_cost(*tournee);
    if(print_alldebug_2opt)cout << "tournee #"<<ibox<<" initial cost = " << initial_cost << endl; 
    //on cherche un 2opt avec meilleur cout
    bool amelioration = true;
    while (amelioration && (*tournee).size() > 0) {
      amelioration = false;
      //pour tout sommet xi de la tournée 
      for(int i = 0; i < tournee->size()-1  && !amelioration ; i++) {
        //pour tout sommet xj de la tournée 
        for(int j = 0; j < tournee->size()-1 && !amelioration; j++) {
          int xi = tournee->at(i); //important de le faire ici pour MAj tout le temps
          int xi_plus1 = tournee->at(i+1);
          int xj = tournee->at(j);
          int xj_plus1 = tournee->at(j+1);
          //(avec j != i-1, i et i+1 car échanger une arete par elle même dans l'autre sens ne change rien)
          if(j != i && j != i-1 && j != i+1) {
            //si l'échange de ces deux arêtes donne une meilleure distance
            float curr_distance = G->get_distance_startat0(xi, xi_plus1) +G->get_distance_startat0(xj, xj_plus1);
            float new_distance  = G->get_distance_startat0(xi, xj)   +G->get_distance_startat0(xi_plus1, xj_plus1); 
            if(new_distance < curr_distance) { //si amélioration
              if(print_alldebug_2opt || print_shortdebug) cout << "2opt improvement found : " << new_distance << " from " << curr_distance << endl;
              //on remplace les arêtes (xi,xi+1) et (xj,xj+1) par (xi,xj) et (xi+1,xj+1) dans la tournée
              // pour ça on inverse l'ordre de parcours dans le vecteur
              //    + on inverse l'ordre de parcours de tous les noeuds entre ces 2 noeuds
              // => remplacer xi+1 par xj (et inversement) et inverser l'ordre des noeuds entre eux
              if(print_alldebug_2opt)cout << "\tbefore : " <<  vectorint_tostring(*tournee) << endl;
              if(print_alldebug_2opt)cout << "\tinversement de (" << xi<<"->"<<xi_plus1 << ") et (" << xj<<"->"<<xj_plus1<<")" << endl;
              tournee->at(i+1) = xj; //il faut faire -1 car on prends pas en compte le sommet 0 ici
              tournee->at(j) = xi_plus1;
              if(print_alldebug_2opt)cout << "\tbetween : " << vectorint_tostring(*tournee) << endl;
              //boucle for qui inverse l'ordre des noeuds entre les deux 
              vector<int> temp;
              //on commence par copier la sous-liste
              for(int z = i+2; z <= j-1; z++) {
                  temp.push_back(tournee->at(z));
              }
              if(print_alldebug_2opt)cout << "\ttemp vector : " << vectorint_tostring(temp) << endl; 
              //maintenant on remplace en inversant
              int counter=0;
              for(int z = i+2; z <= j-1; z++) {
                  tournee->at(z) = temp.at(temp.size()-1-counter);
                  counter++;
              }
              if(print_alldebug_2opt)cout << "\tafter : " << vectorint_tostring(*tournee) << endl;
              if(print_alldebug_2opt || print_shortdebug)cout << "\ttournee #"<<ibox<<" new cost = " << G->get_route_cost(*tournee) << endl; 

              amelioration = true; //et continuer à chercher des améliorations
            }
          }
        }
      }
    }
    cout << "\ttournee #"<<ibox<<"  cost : "<< initial_cost <<" -> " << G->get_route_cost(*tournee) << endl; 
  }
}

void optimize_2opt_switchRoutes(vector<vector<int>> *tournees, C_Graph* G) {
  // voisinages de changement de tournées
  cout << endl <<  "*** starting 2opt optimization : agent switching routes ***"<< endl;

  //////////// PARAMETERS ///////////
  bool print_alldebug_2opt = false;
  float percent_threshold = 0.01; // only improve if it's more than for ex 1% (use 0.01) OR use 0 to improve ALL the time
  float threshold_leaving = 0.01; // 0.01% chance of leaving the function (avoid it running too long for huge instances)
  ///////////////////////////////////
  // pour rendre la fonction moins biaisée sur le fait de ne déplacer que les premiers éléments des premieres tournées
  // -> faire un shuffle des tournées aléatoire au début de la fonction
  // -> ce shuffle est plutôt bénéfique sur les grandes instances et en général, mais peu donner de moins bonnes solutions sur de petites instances si on à pas de chance
  std::random_shuffle ( tournees->begin(), tournees->end() ); // OK VERIFIE

  int nb_box_used = tournees->size();
  int capaciteQ = (*G).VRP_capacity; // capacité max des véhicules
  //if(print_alldebug_2opt) print_all_tournees(*tournees, G);
  bool amelioration = true; // mettre dans toutes les boucles for un && !amelioration pour s'arreter dès la 1ere trouvée
  while(amelioration) {
    std::random_shuffle ( tournees->begin(), tournees->end() ); // OK VERIFIE
    amelioration = false;
    //pour chaque tournée
    float init_total_cost = G->get_VRP_cost(*tournees);
    //cout << "capacitéQ = " << capaciteQ << endl;
    if(print_alldebug_2opt) cout << "init_total_cost = " << init_total_cost << endl;
    for(int ibox = 0; ibox < nb_box_used && !amelioration; ibox++) {
        // on travaille sur une copie des tournées
        vector<vector<int>> copietemp = *tournees; // copie des valeurs (sans pointeurs)
        vector<int> tournee = copietemp.at(ibox);
        //if(print_alldebug_2opt)cout << "tournee #"<<ibox<<" initial cost = " << initial_cost << endl; 
        //pour tout sommet de la tournée 
        for(int i = 0; i < copietemp.at(ibox).size() && !amelioration; i++) {
            vector<int> tourneecopy = copietemp.at(ibox); //on travaille sur une copie des valeurs (sans pointeurs)
            int currentclient = tourneecopy.at(i); // client courant
            float currentclientdemande =  G->get_node_by_id_startat0(currentclient)->VRP_demand;
            //cout << " > client "<<currentclient << "   demande "<<currentclientdemande<< endl;
            //on commence par le supprimer de sa position actuelle
            //cout << "old : " << vectorint_tostring(copietemp.at(ibox)) << endl;
            //cout << "before erase" << endl;
            tourneecopy.erase(tourneecopy.begin()+i); 
            //cout << "new : " << vectorint_tostring(copietemp.at(ibox)) << endl;
            //on regarde si on peut optimiser le cout total en le placant dans une autre tournée ailleurs
            for(int other = 0; other < nb_box_used && !amelioration; other++) {
              //on ne regarde que les autres tournées
              if(other != ibox) {
                  //on vérifie que la capacité permet de l'ajouter dans cette tournée
                  if(G->get_route_demand(copietemp.at(other)) + currentclientdemande <= capaciteQ) {
                      //cout << "\tcould be added to route #"<<other << " of demand : " << G->get_route_demand(copietemp.at(other)) << endl;
                      //on calcule le cout de toutes les autres tournées (pour pouvoir comparer rapidement après)
                      //c'est à dire le cout du VRP initial, moins le vecteur avant erase + le cout du vecteur après erase
                      //cout << "lol"<<endl;
                      float VRPcost = G->get_VRP_cost(copietemp) - G->get_route_cost(copietemp.at(ibox)) + G->get_route_cost(tourneecopy);
                      //cout << "mdr" << endl;
                      float allbutother_cost = VRPcost - G->get_route_cost(copietemp.at(other));
                      //cout << "ptdr" << endl;
                      int const_size = copietemp.at(other).size(); // we will change it so we must make it const here
                      //cout << "XD" << endl;
                      //on va travailler sur une copie du vecteur
                      //maintenant on regarde tous les emplacements dans cette tournée où on pourrait l'ajouter
                      for(int j = 0; j <= const_size && !amelioration; j++) { // <= car on peut ajouter en 1er ou dernier ici
                          //if(const_size == 0) cout << "ALERT EMPTY ROUTE" << endl;
                          vector<int> vectorcopy_temp = copietemp.at(other); // nouvelle copie (sans pointeurs)
                          //vector->insert(iterator pos, const T& value); // insert l'élément value AVANT pos !!
                          //cout << "old2 : " << vectorint_tostring(vectorcopy_temp) << endl;
                          vectorcopy_temp.insert(vectorcopy_temp.begin()+j, currentclient);
                          //cout << "new2 : " << vectorint_tostring(vectorcopy_temp) << endl;
                          //on calcule le cout de cette nouvelle tournée
                          float newroute_cost = G->get_route_cost(vectorcopy_temp);
                          // on regarde si le cout total est meilleur
                          float improvement = init_total_cost - allbutother_cost - newroute_cost;
                          //cout << "improvement of " << init_total_cost-allbutother_cost-newroute_cost << " >? " <<  percent_threshold*init_total_cost << endl; 
                          if(improvement > percent_threshold*init_total_cost) { //2nd version : only if improvement is more than 1%
                            if(print_alldebug_2opt) cout << " > client "<<currentclient << "   demande "<<currentclientdemande<< endl;
                            if(print_alldebug_2opt) cout << "\tMEILLEUR COUT TOTAL de " << allbutother_cost+newroute_cost << " by adding at pos "<<j<<" of route #"<<other  << endl;
                            // alors on remplace les vecteurs pour garder la meilleure solution
                            (*tournees).at(ibox) = tourneecopy;
                            (*tournees).at(other) = vectorcopy_temp;
                            //print_all_tournees(*tournees, G);
                            // et on indique qu'on veut sortir pour recommencer tout depuis le début
                            // car on s'arrête à chaque fois à la première amélioration qu'on trouve (stratégie heuristique)
                            amelioration = true;
                          } else {
                            //cout << "\tmoins bon cout total de "<< allbutother_cost+newroute_cost << " by inserting at pos "<<j<<" of route #"<<other << endl;
                            //print_all_tournees(*tournees, G);
                          }
                          // 0.1% chance of leaving the function (avoid it running too long for huge instances)
                          if(((double) rand() / (RAND_MAX))*100 < threshold_leaving){
                              return;
                          }
                      }
                  }
              }
            }
        }
    }
  }
}

//PLNE utilisant la formulation compacte MTZ
void optimizeMTZ(vector<vector<int>> *tournees, C_Graph* G, string filename){
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
	

  //contraintes requises par le cassage des symétries: en ajoutant x[i,j] = x[j,i] en contrainte
  //TODO il y a peut-être moyen de ne pas avoir besoin de faire ça pour casser la symétrie ? plutot avec i < j ?
  /*
  for (i=0; i < G->nb_nodes; i++) {
    for (j=0; j < G->nb_nodes; j++) { 
      if(i < j) { // i < j pour eviter les contraintes redondantes
        IloExpr csym(env);
        csym = x[i][j] - x[j][i];
        CC.add(csym==0);
        ostringstream nomcst2;
        nomcst2.str("");
        nomcst2<<"Cst_x_"<<i<<"_"<<j<<"="<<"x_"<<j<<"_"<<i;
        cout << nomcst2.str() << endl;
        CC[nbcst].setName(nomcst2.str().c_str());
        nbcst++;
        
      }
    }
  }
  */
	
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
        //obj.setLinearCoef(x[i][j], G->lengthTSP(i,j));
        obj.setLinearCoef(x[i][j], 0); // affiche la première solution trouvée valide
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
  cout<<"---"<<endl;
  for(int i = 1; i < G->nb_nodes; i++) {
    cout <<"w_" << i<<" = " << cplex.getValue(w[i]) << endl;
  }
  cout<<"---"<<endl;

  // on enregistre tous les arcs du graphe dans sol
  // et on va aussi exporter la solution dans le pointeur d'entrée "tournees"
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
  cout << "solution MTZ par tournées : "<< endl;
  bool display_debug_mtz = false;
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


int main (int argc, char**argv){
  //////////////////////////////
  ///////// PARAMETERS /////////
  //////////////////////////////
  bool relaxedPLNE_activateprint  = false;
  bool relaxedPLNE_activateoutput = false;
  ////////////////////////////// end of parameters

  std::srand(std::time(0)); // seed for random,very important

  vector<int> solution_vec;

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" filename.vrp"<<endl; 
    return 1;
  }
  string filename = argv[1];

  C_Graph* G = parseVRPfile(filename, relaxedPLNE_activateprint);
  //C_node* cnode = graph_ptr->get_node_by_id(1);    //exemple d'utilisation
  //(*G).nb_nodes;    // G->nb_nodes;                //both mean the same here
  //C_node* cnode  = (*G).get_node_by_id_startat1(1);   //example
  int capaciteQ = (*G).VRP_capacity; // capacité max des véhicules

  //on solve par PLNE après avoir relaxé la contrainte de m tournées
  int nb_box_used = solve_relaxedPLNE(G, filename, &solution_vec, relaxedPLNE_activateprint, relaxedPLNE_activateoutput); //last paramter is write_outputs?, befre-last is print?

  cout << endl;
  //print solution found by relaxed PLNE
  cout << "NB BOXES USED : " << nb_box_used << endl;
  cout << "SOLUTION : ";
  for(int i = 0; i < solution_vec.size(); i++)
        cout << solution_vec.at(i) << " " ;
  cout << endl;
  // remarque : la solution trouvée ne contient que des valeurs qui se suivent à partir de 1 : 1,2,3,4,5...

  /////////////////////////////////////////////////////////////////////////////////////
  //////////////// métaheuristique itérative par voisinage ////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
  /*
    A l’issue d’une étape initiale, on obtient une solution réalisable.
    Une métaheuristique itérative peut alors être utilisée sur la base de plusieurs voisinages:
    - les voisinages classiques du TSP (2-opt) pour améliorer chaque tournée indépendemment.
    - la possiblité pour un client de changer de tournées
    - supprimer une tournée vide
    - éventuellement ajouter une tournée vide 
    OBj: minimiser la somme toale des couts des arcs utilisés (sous contrainte de capacité des véhicules)
    rmq: on peut décrire une tournée par une liste de sommets (i0, i1, ...ip) comme le graphe est complet.
         tous les circuits commencent par le sommet 0 et finissent au sommet 0
    TODO: -> peut-être ajouter métaheuristique en ajoutant une tournée vide au début qu'on supprime ensuite ?-> testé mais change rien ?
  */
  //création du vecteur de vecteurs contenant toutes les tournées (sans le sommet 0):
  vector<vector<int>> tournees;
  //pour chacune des tournées
  for(int i = 0; i < nb_box_used; i++) {
      vector<int> curr_tournee;
      //on lit tout le vecteur solution
      for(int j = 0; j < solution_vec.size(); j++){
          //on extrait les sommets de la tournée à enregistrer
          if(solution_vec.at(j) == i+1) {
              curr_tournee.push_back(j+1); //+1 pour commencer à 1 au lieu de 0
          }
      }
      std::random_shuffle ( curr_tournee.begin(), curr_tournee.end() );  //shuffle optimise bcp la vitesse moyenne !
      tournees.push_back(curr_tournee);
  }
  std::random_shuffle(tournees.begin(), tournees.end());//shuffle optimise bcp la vitesse moyenne !
  //test : ajout d'une tournée vide
  //vector<int> tournee_vide;
  //tournees.push_back(tournee_vide);

  //on affiche toutes les tournées et leurs couts:
  cout<<endl; print_all_tournees(tournees, G);

  float previous_best_VRPcost = G->get_VRP_cost(tournees);
  float current_best_VRPcost  = G->get_VRP_cost(tournees);
  //tant qu'on arrive à améliorer par 2opt on continue

  //TODO a decommenter une fois qu'on a finit de tester MTZ
  /*
  int nb_stagnations = 0; // use 10 for a pretty exhaustive search
  do {
    if(previous_best_VRPcost <= current_best_VRPcost) nb_stagnations++;
    else nb_stagnations = 0;

    //voisinage 2opt pour optimiser chaque tournée indépendament
    optimize_2opt_internalRoutes(&tournees, G);
    print_all_tournees(tournees, G);

    //voisinage 2opt pour optimiser entre les tournées
    optimize_2opt_switchRoutes(&tournees, G);
    print_all_tournees(tournees, G);

    //MAJ les valeurs
    previous_best_VRPcost = current_best_VRPcost;
    current_best_VRPcost = G->get_VRP_cost(tournees);
  }while (nb_stagnations < 10);		// si on voit que 10x de suite on à le même cout on arrête  

  //on affiche toutes les tournées et leurs couts:
  cout << "after 2opt optimization : " << endl;
  
  cout<<endl; print_all_tournees(tournees, G);
  */
  
  ///////////////////////////////////////////////////////////////////////////////////// fin métaheuristique itérative par voisinage
  
  optimizeMTZ(&tournees, G, filename);
  
  //on affiche toutes les tournées et leurs couts:
  //cout << "after MTZ optimization : " << endl;
  cout<<endl; print_all_tournees(tournees, G);
  
  return 0;
}
