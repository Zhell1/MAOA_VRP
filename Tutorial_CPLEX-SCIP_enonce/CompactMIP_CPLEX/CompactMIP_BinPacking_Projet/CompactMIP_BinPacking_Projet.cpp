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
  bool activateoutput = true;
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
  cout << "ND BOXES USED : " << nb_box_used << endl;
  cout << "SOLUTION : ";
  for(int i = 0; i < solution_vec.size(); i++)
        cout << solution_vec.at(i) << " " ;
  cout << endl;

  /////////////////////////////////////////////////////////////////////////////////////
  //////////////// métaheuristique itérative par voisinage ////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////

  /*
    A l’issue d’une étape initiale, on obtient une solution réalisable ou alors une solution utilisant
    plus de m tournées (chacune réalisable).
    Une métaheuristique itérative peut alors être utilisée sur la base de plusieurs voisinages:
    - les voisinages classiques du TSP (2-opt) pour améliorer chaque tournée indépendemment.
    - la possiblité pour un client de changer de tournées
    - supprimer une tournée vide
    - éventuellement ajouter une tournée vide si le nombre est inférieur à m
    Si le nombre de tournées de la solution initiale est supérieur à m, il est possible d’utiliser
    dans un premier temps une fonction objective artificielle pour forcer à réduire le nombre de
    tournées.
  */

  //TODO

  ///////////////////////////////////////////////////////////////////////////////////// fin métaheuristique itérative par voisinage
  

  return 0;
}
