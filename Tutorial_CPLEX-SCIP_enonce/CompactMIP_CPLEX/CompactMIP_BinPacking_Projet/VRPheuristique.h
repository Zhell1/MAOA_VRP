#ifndef VRPHEURISTIQUE_H
#define VRPHEURSTIQUE_H

#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include"../../Graph/Graph.h"
#include "./VRPfileParser.h"
#include "./solve_relaxedPLNE.h"
//mtz_plne.cpp contains the basic mtz without reinforcing
#include "./VRPtools.h"
//  string vectorint_tostring(vector<int> my_vector);
//  void write_solution_to_file(vector<vector<int>> tournees, C_Graph* G, string filename);
//  void print_all_tournees(vector<vector<int>> tournees, C_Graph* G);


#define epsilon 0.00001

using namespace std;

bool shuffling_active = true;

void optimize_2opt_internalRoutes(vector<vector<int>> *tournees, C_Graph* G) {
   //voisinage 2opt pour optimiser chaque tournée indépendament
  cout << endl <<  "*** starting 2opt optimization of internal routes ***"<< endl;

  bool print_alldebug_2opt = false;
  bool print_shortdebug = false; //prints less stuff

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
    if(print_alldebug_2opt || print_shortdebug) cout << "\ttournee #"<<ibox<<"  cost : "<< initial_cost <<" -> " << G->get_route_cost(*tournee) << endl; 
  }
}

void optimize_2opt_switchRoutes(vector<vector<int>> *tournees, C_Graph* G) {
  // voisinages de changement de tournées
  cout << endl <<  "*** starting 2opt optimization : agent switching routes ***"<< endl;

  //////////// PARAMETERS ///////////
  bool print_alldebug_2opt = false;
  float percent_threshold = 0.0; // only improve if it's more than for ex 1% (use 0.01) OR use 0 to improve ALL the time
  float threshold_leaving = 0.01; // 0.01% chance of leaving the function (avoid it running too long for huge instances)
  ///////////////////////////////////
  // pour rendre la fonction moins biaisée sur le fait de ne déplacer que les premiers éléments des premieres tournées
  // -> faire un shuffle des tournées aléatoire au début de la fonction
  // -> ce shuffle est plutôt bénéfique sur les grandes instances et en général, mais peu donner de moins bonnes solutions sur de petites instances si on à pas de chance
  if(shuffling_active) std::random_shuffle ( tournees->begin(), tournees->end() ); // OK VERIFIE

  int nb_box_used = tournees->size();
  int capaciteQ = (*G).VRP_capacity; // capacité max des véhicules
  //if(print_alldebug_2opt) print_all_tournees(*tournees, G);
  bool amelioration = true; // mettre dans toutes les boucles for un && !amelioration pour s'arreter dès la 1ere trouvée
  while(amelioration) {
    if(shuffling_active) std::random_shuffle ( tournees->begin(), tournees->end() ); // OK VERIFIE
    amelioration = false;
    //pour chaque tournée
    float init_total_cost = G->get_VRP_cost(*tournees);
    //cout << "capacitéQ = " << capaciteQ << endl;
    if(print_alldebug_2opt) cout << "init_total_cost = " << init_total_cost << endl;
    for(int ibox = 0; ibox < nb_box_used && !amelioration; ibox++) {
        // on travaille sur une copie des tournées
        vector<vector<int>> copietemp = *tournees; // copie des valeurs (sans pointeurs)
        vector<int> tournee = copietemp.at(ibox);
    //    if(print_alldebug_2opt)cout << "tournee #"<<ibox<<" initial cost = " << initial_cost << endl; 
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
#endif