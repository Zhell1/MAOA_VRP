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

  int nb_box_used = tournees->size();
  //on parcours toutes les tournées une par une : 
  for(int ibox = 0; ibox < nb_box_used; ibox++) {
    vector<int> *tournee = &(tournees->at(ibox)); // passe par un pointeur pour pouvoir modifier direct les valeurs
    //tournee->at(0) = 1000; cout << tournees.at(i).at(0) << " ???"<< endl; // test de modification OK
    float initial_cost = G->get_route_cost(*tournee);
    if(print_alldebug_2opt)cout << "tournee #"<<ibox<<" initial cost = " << initial_cost << endl; 
    //on cherche un 2opt avec meilleur cout
    bool amelioration = true;
    while (amelioration) {
      amelioration = false;
      //pour tout sommet xi de la tournée 
      for(int i = 0; i < tournee->size()-1; i++) {
        //pour tout sommet xj de la tournée 
        for(int j = 0; j < tournee->size()-1; j++) {
          int xi = tournee->at(i); //important de le faire ici pour MAJ tout le temps
          int xi_plus1 = tournee->at(i+1);
          int xj = tournee->at(j);
          int xj_plus1 = tournee->at(j+1);
          //(avec j != i-1, i et i+1 car échanger une arete par elle même dans l'autre sens ne change rien)
          if(j != i && j != i-1 && j != i+1) {
            //si l'échange de ces deux arêtes donne une meilleure distance
            float curr_distance = G->get_distance_startat0(xi, xi_plus1) +G->get_distance_startat0(xj, xj_plus1);
            float new_distance  = G->get_distance_startat0(xi, xj)   +G->get_distance_startat0(xi_plus1, xj_plus1); 
            if(new_distance < curr_distance) { //si amélioration
              if(print_alldebug_2opt) cout << "\t2opt improvement found : " << new_distance << " from " << curr_distance << endl;
              //on remplace les arêtes (xi,xi+1) et (xj,xj+1) par (xi,xj) et (xi+1,xj+1) dans la tournée
              // pour ça on inverse l'ordre de parcours dans le vecteur
              //    + on inverse l'ordre de parcours de tous les noeuds entre ces 2 noeuds
              // => remplacer xi+1 par xj (et inversement) et inverser l'ordre des noeuds entre eux
              if(print_alldebug_2opt)cout << "\t\tbefore : " <<  vectorint_tostring(*tournee) << endl;
              if(print_alldebug_2opt)cout << "\t\tinversement de (" << xi<<"->"<<xi_plus1 << ") et (" << xj<<"->"<<xj_plus1<<")" << endl;
              tournee->at(i+1) = xj; //il faut faire -1 car on prends pas en compte le sommet 0 ici
              tournee->at(j) = xi_plus1;
              if(print_alldebug_2opt)cout << "\t\tbetween : " << vectorint_tostring(*tournee) << endl;
              //boucle for qui inverse l'ordre des noeuds entre les deux 
              vector<int> temp;
              //on commence par copier la sous-liste
              for(int z = i+2; z <= j-1; z++) {
                  temp.push_back(tournee->at(z));
              }
              if(print_alldebug_2opt)cout << "\t\ttemp vector : " << vectorint_tostring(temp) << endl; 
              //maintenant on remplace en inversant
              int counter=0;
              for(int z = i+2; z <= j-1; z++) {
                  tournee->at(z) = temp.at(temp.size()-1-counter);
                  counter++;
              }
              if(print_alldebug_2opt)cout << "\t\tafter : " << vectorint_tostring(*tournee) << endl;
              if(print_alldebug_2opt)cout << "\t\ttournee #"<<ibox<<" new cost = " << G->get_route_cost(*tournee) << endl; 

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

  bool print_alldebug_2opt = false;

  int nb_box_used = tournees->size();
  int capaciteQ = (*G).VRP_capacity; // capacité max des véhicules
  if(print_alldebug_2opt) print_all_tournees(*tournees, G);
  bool amelioration = true; // mettre dans toutes les boucles for un && !amelioration pour s'arreter dès la 1ere trouvée
  while(amelioration) {
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
                      float VRPcost = G->get_VRP_cost(copietemp) - G->get_route_cost(copietemp.at(ibox)) + G->get_route_cost(tourneecopy);
                      float allbutother_cost = VRPcost - G->get_route_cost(copietemp.at(other));
                      int const_size =copietemp.at(other).size(); // we will change it so we must make it const here
                      //on va travailler sur une copie du vecteur
                      //maintenant on regarde tous les emplacements dans cette tournée où on pourrait l'ajouter
                      for(int j = 0; j <= const_size && !amelioration; j++) { // <= car on peut ajouter en 1er ou dernier ici
                          vector<int> vectorcopy_temp = copietemp.at(other); // nouvelle copie (sans pointeurs)
                          //vector->insert(iterator pos, const T& value); // insert l'élément value AVANT pos !!
                          //cout << "old2 : " << vectorint_tostring(vectorcopy_temp) << endl;
                          vectorcopy_temp.insert(vectorcopy_temp.begin()+j, currentclient);
                          //cout << "new2 : " << vectorint_tostring(vectorcopy_temp) << endl;
                          //on calcule le cout de cette nouvelle tournée
                          float newroute_cost = G->get_route_cost(vectorcopy_temp);
                          // on regarde si le cout total est meilleur
                          if(allbutother_cost+newroute_cost < init_total_cost) {
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
                      }
                  }
              }
            }
        }
    }
  }
}

int main (int argc, char**argv){
  //////////////////////////////
  ///////// PARAMETERS /////////
  //////////////////////////////
  bool relaxedPLNE_activateprint  = false;
  bool relaxedPLNE_activateoutput = false;
  ////////////////////////////// end of parameters

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
    OBJ: minimiser la somme toale des couts des arcs utilisés (sous contrainte de capacité des véhicules)
    rmq: on peut décrire une tournée par une liste de sommets (i0, i1, ...ip) comme le graphe est complet.
         tous les circuits commencent par le sommet 0 et finissent au sommet 0
    TODO: -> peut-être ajouter métaheuristique en ajoutant une tournée vide au début qu'on supprime ensuite ?
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
      tournees.push_back(curr_tournee);
  }
  //on affiche toutes les tournées et leurs couts:
  cout<<endl; print_all_tournees(tournees, G);

  float previous_best_VRPcost = G->get_VRP_cost(tournees);
  float current_best_VRPcost  = G->get_VRP_cost(tournees);
  bool firsttime =true; //pour lancer la boucle while au moins 1 fois
  //tant qu'on arrive à améliorer par 2opt on continue
  while (firsttime || previous_best_VRPcost > current_best_VRPcost) {
    firsttime = false;
    //voisinage 2opt pour optimiser chaque tournée indépendament
    optimize_2opt_internalRoutes(&tournees, G);
    print_all_tournees(tournees, G);

    //voisinage 2opt pour optimiser entre les tournées
    optimize_2opt_switchRoutes(&tournees, G);
    print_all_tournees(tournees, G);

    //MAJ les valeurs
    previous_best_VRPcost = current_best_VRPcost;
    current_best_VRPcost = G->get_VRP_cost(tournees);
  }

  //on affiche toutes les tournées et leurs couts:
  cout << "after 2opt optimization : " << endl;
  cout<<endl; print_all_tournees(tournees, G);

  ///////////////////////////////////////////////////////////////////////////////////// fin métaheuristique itérative par voisinage
  
  /*
  thomas @Baptiste:
        tu peux repartir de la solution dans "tournees" comme solution initiale (borne) si c'est utile?
           pour faire la partie 1.0.2 : Formulations PLNE pour le VRP

        attention dans nos instances les couts sont symétriques donc il faut changer les
          formulations du sujet par des variables non-orientées !!
  */

  return 0;
}
