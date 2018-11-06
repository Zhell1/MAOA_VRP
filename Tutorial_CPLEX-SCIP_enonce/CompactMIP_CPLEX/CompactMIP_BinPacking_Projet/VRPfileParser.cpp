#include <stdio.h>      /* printf */
#include <math.h>       /* sqrt */

#include <iostream>
#include <iomanip>
#include <string.h>
#include <sstream>

#include <iostream>
#include <fstream>

#include "../../Graph/Graph.h"

using namespace std;

///  load a .vrp file into a Graph Object

#define epsilon 0.00001
#define MAXLINE 256

//functions of this file
C_Graph* parseVRPfile(string filename);
float EUC_2D (int x1, int y1, int x2, int y2);

//
int main (int argc, char**argv)
{
	if(argc!=2){
		cerr<<"usage: "<<argv[0]<<" filename.vrp"<<endl; 
		return 1;
	}

	string filename = argv[1];

	parseVRPfile(filename);

	return 0;
}

C_Graph* parseVRPfile(string filename)
{

   C_Graph* mygraph = new C_Graph();
   mygraph->directed = false; //normalement à true mais les instances ont toutes des couts symmétriques

   cout << "*** STARTS READING FILE : " << filename << std::endl;

   //ifstream inFile(filename, ios::in);
    ifstream inFile;
    inFile.open(filename);
    if(inFile.fail()) {
    	cout << "ERROR FILE NOT FOUND !!!!" << endl;
    	return nullptr;
    }
    
	char oneline[MAXLINE];
	bool reading_nodecoordsection = false;
	bool reading_demandsection = false;

	while (inFile)
	{

		inFile.getline(oneline, sizeof(oneline));
		std::stringstream sstream(oneline);
		//cout << oneline << endl;

		string word;
		if( !reading_nodecoordsection && !reading_demandsection ){
			sstream >> word;
			// cout << word << endl;
		}

		if(strcmp(word.c_str(), "DIMENSION") == 0) {
		    sstream >> word; // ':'
		    sstream >> mygraph->nb_nodes; // int value  
		    cout <<  mygraph->nb_nodes << " noeuds" << endl; 
		    int n = mygraph->nb_nodes;
		    if(mygraph->directed == true){
			    mygraph->nb_links = n * (n - 1) ; // pas /2 comme c'est des arcs	   
			    cout <<  mygraph->nb_links << " arcs" << endl; 
			}
		    else {
			    mygraph->nb_links = n * (n - 1) /2 ;  
			    cout <<  mygraph->nb_links << " aretes" << endl; 
		    }

		    mygraph->V_nodes.resize(mygraph->nb_nodes);
			mygraph->V_links.resize(mygraph->nb_links);

			for (int i=0 ; i < mygraph->nb_nodes ;i++){ 
		      mygraph->V_nodes[i].num = i+1;				//file starts at 1, not 0
		      mygraph->V_nodes[i].L_adjLinks.clear();
		      mygraph->V_nodes[i].weight=1;					// pas de poids sur les noeuds pour nous
		    }
		    
		}
		else if(strcmp(word.c_str(), "CAPACITY") == 0) {
		    sstream >> word; // ':'
		    sstream >> mygraph->VRP_capacity; // int value  
		    cout <<  mygraph->nb_nodes << " de capacité par véhicule" << endl; 	  
		}
		else if(strcmp(word.c_str(), "NODE_COORD_SECTION") == 0) {
			reading_nodecoordsection = true;
			cout << "NODE_COORD_SECTION" << endl;
		}
		else if (reading_nodecoordsection == true) {
			int id = 0;
			sstream >> id;	//id

			//!\\ /!\																		/!\
			//!\\ /!\  mygraph->get_node_by_id(id)  =   mygraph->V_nodes[id-1] 				/!\
			//!\\ /!\																		/!\
			//!\\ /!\                 using get_node_by_id() is safer						/!\
			//!\\ /!\			         because files starts at 1							/!\
			//!\\ /!\																		/!\

			sstream >> mygraph->get_node_by_id(id)->x;	//x
			sstream >> mygraph->get_node_by_id(id)->y;	//y
			cout << "lecture coordonnée : noeud " << id << " at (" << mygraph->get_node_by_id(id)->x << ", " << mygraph->get_node_by_id(id)->y << ")" << endl;

			if (id == mygraph->nb_nodes) {
				reading_nodecoordsection = false;	//so that we can read the next section title after
				//OK VÉRIFIE
			}
		}
		
		else if(strcmp(word.c_str(), "DEMAND_SECTION") == 0) {
			reading_demandsection = true;
			cout << "DEMAND_SECTION" << endl;
		}
		else if (reading_demandsection == true){
			int id = 0;
			sstream >> id;
			sstream >> mygraph->get_node_by_id(id)->VRP_demand;

			cout << "lecture demande : noeud " << id << " demands " << mygraph->get_node_by_id(id)->VRP_demand << endl;

			if (id == mygraph->nb_nodes) {
				reading_demandsection = false;	//so that we can read the next section title after
				//OK VÉRIFIE
			}
		}
		else {
			//do nothing
		}       
	}
	cout << "END OF FILE READING" << endl;

	C_link *a;
	int k = 0;
	C_node *noeudi, *noeudj;

	for (int i=0 ; i < mygraph->nb_nodes ; i++){
		cout << "adding links : ";
		for(int j=i+1 ; j < mygraph->nb_nodes ; j++) { // double boucle pour graphe complet
			a = new C_link;
			a->num = k;
			noeudi = &(mygraph->V_nodes[i]); // V_nodes[i] because we use int i = 0 to start
			noeudj = &(mygraph->V_nodes[j]);
			cout << noeudi->num << "-" << noeudj->num << "\t";
			a->v1 = noeudi->num -1; // OK VERIF
			a->v2 = noeudj->num -1; // OK VERIF
			a->length = EUC_2D(noeudi->x, noeudi->y, noeudj->x, noeudj->y); // why not ? it's cheap
			mygraph->V_nodes[i].L_adjLinks.push_back(a);
			mygraph->V_nodes[j].L_adjLinks.push_back(a); //si on met dans les sens  = version non dirigée Undirected, sinon changer, voir code dans C_graph.cpp
			mygraph->V_links[k] = a;
			k++;
		}
		cout << endl;
	}

	cout << k <<" LINKS CONSTRUITS" << endl;

	if(mygraph->directed == false)	
		mygraph->construct_Undirected_Lemon_Graph();
	else 
		mygraph->construct_Directed_Lemon_Graph();

	cout << "LEMON OK" << endl;

	inFile.close();

	cout << "*** FILE CLOSED" << endl;

	return mygraph;
}


//calcule la distance euclidienne dans un monde en 2D
float EUC_2D (int x1, int y1, int x2, int y2){
	return sqrt((x1 - x2)^2 + (y1 - y2)^2);
}