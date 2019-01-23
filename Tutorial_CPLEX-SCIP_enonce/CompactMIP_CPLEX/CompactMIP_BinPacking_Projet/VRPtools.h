#ifndef VRPTOOLS_H
#define VRPTOOLS_H

#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include"../../Graph/Graph.h"
#include "./VRPfileParser.h"
#include "./solve_relaxedPLNE.h"

#define epsilon 0.00001s

using namespace std;

string vectorint_tostring(vector<int> my_vector) {
  std::stringstream result;
  std::copy(my_vector.begin(), my_vector.end(), std::ostream_iterator<int>(result, " "));
  return result.str().c_str();
}

void write_solution_to_file(vector<vector<int>> tournees, C_Graph* G, string filename){
	C_Graph* mygraph = new C_Graph();
	mygraph->directed = false;//G->directed;
	mygraph->nb_nodes = G->nb_nodes;
	
	int n = 0;
	for(auto it = tournees.begin(); it != tournees.end(); ++it) {
		n += (*it).size() + 1; // le noeud 0 n'est pas dans le vecteur
	}
	mygraph->nb_links = n;
	
	mygraph->V_nodes.resize(mygraph->nb_nodes);
	mygraph->V_links.resize(mygraph->nb_links);

	for (int i=0 ; i < mygraph->nb_nodes ;i++){ 
		mygraph->V_nodes[i].num = i;				
		mygraph->V_nodes[i].L_adjLinks.clear();
		mygraph->V_nodes[i].weight=1;				// pas de poids sur les noeuds pour nous
		mygraph->V_nodes[i].x = G->V_nodes[i].x;
		mygraph->V_nodes[i].y = G->V_nodes[i].y;
		mygraph->V_nodes[i].VRP_demand = G->V_nodes[i].VRP_demand;
	}
	
	C_link *a;
	int k = 0;
	C_node *noeudi, *noeudj;
	bool print_links = false; // make this true to print all the links as they are being added
	
	vector<int> coloring;
	coloring.resize(G->nb_nodes);
	coloring[0] = 0;
	
	//ajout des liens
	for(auto it = tournees.begin(); it != tournees.end(); ++it) {
		if(print_links) cout << "adding link : 0 -> " << (*(it))[0] << endl;
		a = new C_link;
		a->num = k;
		noeudi = &(mygraph->V_nodes[0]); // V_nodes[i] because we use int i = 0 to start
		noeudj = &(mygraph->V_nodes[(*(it))[0]]);
		a->v1 = noeudi->num ; // OK VERIF
		a->v2 = noeudj->num ; // OK VERIF
		//a->length = EUC_2D(noeudi->x, noeudi->y, noeudj->x, noeudj->y); // why not ? it's cheap
		mygraph->V_nodes[0].L_adjLinks.push_back(a);
		mygraph->V_nodes[(*(it))[0]].L_adjLinks.push_back(a); //si on met dans les sens  = version non dirigée Undirected, sinon changer, voir code dans C_graph.cpp
		mygraph->V_links[k] = a;
		k++;
		
		coloring[*((*it).begin())] = it - tournees.begin() + 1;
		
		for(auto node = (*it).begin() + 1; node != (*it).end(); ++node) {
			
			if(print_links) cout << "adding link : " << (*(node - 1)) << " -> " << *(node) << endl;
			a = new C_link;
			a->num = k;
			noeudi = &(mygraph->V_nodes[(*(node - 1))]); // V_nodes[i] because we use int i = 0 to start
			noeudj = &(mygraph->V_nodes[*(node)]);
			a->v1 = noeudi->num ; // OK VERIF
			a->v2 = noeudj->num ; // OK VERIF
			//a->length = EUC_2D(noeudi->x, noeudi->y, noeudj->x, noeudj->y); // why not ? it's cheap
			mygraph->V_nodes[*(node - 1)].L_adjLinks.push_back(a);
			mygraph->V_nodes[*node].L_adjLinks.push_back(a); //si on met dans les sens  = version non dirigée Undirected, sinon changer, voir code dans C_graph.cpp
			mygraph->V_links[k] = a;
			k++;
				
			coloring[*node] = it - tournees.begin() + 1;
		}
		
		if(print_links) cout << "adding link : " << *((*it).end() - 1) << " -> 0" << endl;
		a = new C_link;
		a->num = k;
		noeudi = &(mygraph->V_nodes[*((*it).end() - 1)]); // V_nodes[i] because we use int i = 0 to start
		noeudj = &(mygraph->V_nodes[0]);
		a->v1 = noeudi->num ; // OK VERIF
		a->v2 = noeudj->num ; // OK VERIF
		//a->length = EUC_2D(noeudi->x, noeudi->y, noeudj->x, noeudj->y); // why not ? it's cheap
		mygraph->V_nodes[*((*it).end() - 1)].L_adjLinks.push_back(a);
		mygraph->V_nodes[0].L_adjLinks.push_back(a); //si on met dans les sens  = version non dirigée Undirected, sinon changer, voir code dans C_graph.cpp
		mygraph->V_links[k] = a;
		k++;
	}
	
	mygraph->write_dot_G_color_svg(filename, coloring);
	cout << "Wrote solution to " << filename << "_G_color.svg" << endl;
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




C_Graph* intsol_to_undirected_C_Graph(vector<vector<int>> arcs, C_Graph* G){
	C_Graph* mygraph = new C_Graph();
	mygraph->directed = false;//G->directed;
	mygraph->nb_nodes = G->nb_nodes; 

	//we remove the symetries from arcs because we will call undirected_mincut later
	for(int i=0; i < G->nb_nodes; i++) {
		for(int j=0; j < G->nb_nodes; j++) {
			if(arcs[i][j] == arcs[j][i]) {
				arcs[j][i] = 0;
			}
		}
	}
	
	//compter le nombre d'arcs à ajouter dans le graphe
	int n = 0;
	for(int i=0; i < G->nb_nodes; i++) {
		for(int j=0; j < G->nb_nodes; j++) {
			if(arcs[i][j] > 0) 
				n++;
		}
	}

	mygraph->nb_links = n;
	
	mygraph->V_nodes.resize(mygraph->nb_nodes);
	mygraph->V_links.resize(mygraph->nb_links);


	for (int i=0 ; i < mygraph->nb_nodes ;i++){ 
		mygraph->V_nodes[i].num = i;				
		mygraph->V_nodes[i].L_adjLinks.clear();
		mygraph->V_nodes[i].weight=1;				// pas de poids sur les noeuds pour nous
		mygraph->V_nodes[i].x = G->V_nodes[i].x;
		mygraph->V_nodes[i].y = G->V_nodes[i].y;
		mygraph->V_nodes[i].VRP_demand = G->V_nodes[i].VRP_demand;
	}
	
	C_link *a;
	int k = 0;
	C_node *noeudi, *noeudj;
	bool print_links = false; // make this true to print all the links as they are being added
	
	//ajout des liens
	for(int i=0; i < mygraph->nb_nodes; i++) {
		for(int j=0; j < mygraph->nb_nodes; j++) {
			if(arcs[i][j] > 0) {
				if(print_links) cout << "adding undirected link : " << i << " <-> " << j << " of "<<mygraph->nb_nodes<<" nodes"<<endl;
				a = new C_link;
				a->num = k;
				noeudi = &(mygraph->V_nodes[i]); // V_nodes[i] because we use int i = 0 to start
				noeudj = &(mygraph->V_nodes[j]);
				a->v1 = min(noeudi->num, noeudj->num);
				a->v2 = max(noeudi->num ,noeudj->num); 
				//a->length = EUC_2D(noeudi->x, noeudi->y, noeudj->x, noeudj->y); // why not ? it's cheap
				mygraph->V_nodes[i].L_adjLinks.push_back(a);
				mygraph->V_nodes[j].L_adjLinks.push_back(a); //si on met dans les sens  = version non dirigée Undirected, sinon changer, voir code dans C_graph.cpp
				mygraph->V_links[k] = a;
				//a->length = EUC_2D(noeudi->x, noeudi->y, noeudj->x, noeudj->y);
				//cout << G->V_links[k]->length << endl;
				a->algo_cost = 1;
				k++;
			}
		}
	}

	//cout << k << " / " <<mygraph->nb_links << endl;

  	mygraph->construct_Undirected_Lemon_Graph();

	return mygraph;
}


#endif