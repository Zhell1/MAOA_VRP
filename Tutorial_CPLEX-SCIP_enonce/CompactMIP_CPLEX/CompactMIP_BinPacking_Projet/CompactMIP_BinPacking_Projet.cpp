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

int main (int argc, char**argv){

  //////////////////////////////
  ///////// PARAMETERS /////////
  //////////////////////////////
  bool activateprint  = true;
  bool activateoutput = false;
  ////////////////////////////// end of parameters

  vector<int> solution_vec;

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" filename.vrp"<<endl; 
    return 1;
  }
  string filename = argv[1];

  C_Graph* G = parseVRPfile(filename, activateprint);
  //C_node* cnode = graph_ptr->get_node_by_id(1);    //exemple d'utilisation
  //(*G).nb_nodes;    // G->nb_nodes;                //both mean the same here
  //C_node* cnode  = (*G).get_node_by_id_startat1(1);   //example

  //on solve par PLNE après avoir relaxé la contrainte de m tournées
  int nb_box_used = solve_relaxedPLNE(G, filename, &solution_vec, activateprint, activateoutput); //last paramter is write_outputs?, befre-last is print?

  //print solution found by relaxed PLNE
  cout << "NB BOXES USED : " << nb_box_used << endl;
  cout << "SOLUTION : ";
  for(int i = 0; i < solution_vec.size(); i++)
        cout << solution_vec.at(i) << " " ;
  cout << endl;

  // la solution trouvée ne contient que des valeurs qui se suivent à partir de 1 : 1,2,3,4,5...

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

        -> fonction: parcourir les tournées de la solution une par une et selectionner le premier mouvement améliorant la tournée
              jusqu'à ce qu'il n'y en ait plus.
              https://fr.wikipedia.org/wiki/2-opt
          
        -> fonction : voisinage "possibilité pour un client de changer de tournées"
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




  ///////////////////////////////////////////////////////////////////////////////////// fin métaheuristique itérative par voisinage
  

  return 0;
}
