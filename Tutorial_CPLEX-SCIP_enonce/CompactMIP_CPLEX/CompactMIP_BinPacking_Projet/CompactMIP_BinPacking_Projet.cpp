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
      cout << "\t tournée #"<<i<< " (cost= "<< G->get_route_cost(tournees.at(i)) << ") : ";
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
  int nb_box_used = tournees->size();
   //voisinage 2opt pour optimiser chaque tournée indépendament
  cout << endl <<  "*** starting 2opt optimization of internal routes ***"<< endl;
  bool print_alldebug_2opt = false;

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

    TODO:
        on peut décrire une tournée par une liste de sommets (i0, i1, ...ip) comme le graphe est complet.
        tous les circuits commencent par le sommet 0 et finissent au sommet 0
          
        -> fonction : voisinage "possibilité pour un client de changer de tournées"
        -> alterner entre les 2 heuristiques précédentes jusqu'à ne plus avoir d'amélioration (mais commencer par le client qui change de tournée)
        -> peut-être ajouter métaheuristique en ajoutant une tournée vide au début qu'on supprime ensuite ?
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

  //voisinage 2opt pour optimiser chaque tournée indépendament
  optimize_2opt_internalRoutes(&tournees, G);
  
  //on affiche toutes les tournées et leurs couts:
  cout<<endl; print_all_tournees(tournees, G);

  ///////////////////////////////////////////////////////////////////////////////////// fin métaheuristique itérative par voisinage
  

  /*
  thomas @Baptiste:
        tu peux repartir de la solution dans "tournees" comme solution initiale (borne) 
           pour faire la partie 1.0.2 Formulations PLNE pour le VRP

        attention dans nos instances les couts sont symétriques donc il faut changer les
          formulations du sujet par des variables non-orientées !!
  */

  return 0;
}
