#ifndef VRPFILEPARSER_H
#define VRPFILEPARSER_H

#include <stdio.h>      /* printf */
#include <math.h>       /* sqrt */

#include <iostream>
#include <iomanip>
#include <string.h>
#include <sstream>
#include <stdlib.h>     /* atof */

#include <iostream>
#include <fstream>

#include "../../Graph/Graph.h"

using namespace std;

///  loads a .vrp file into a Graph Object

#define epsilon 0.00001
#define MAXLINE 256

//functions of this file
C_Graph* parseVRPfile(string filename);
float EUC_2D (int x1, int y1, int x2, int y2);


/*
// main to test the parser
int main (int argc, char**argv)
{
	if(argc!=2){
		cerr<<"usage: "<<argv[0]<<" filename.vrp"<<endl; 
		return 1;
	}

	string filename = argv[1];

	C_Graph* graph = parseVRPfile(filename);

	//C_node* cnode = graph->get_node_by_id(1);  	//exemple d'utilisation

	return 0;
}
*/

///  loads a .vrp file into a Graph Object
C_Graph* parseVRPfile(string filename, bool activateprint)
{

   C_Graph* mygraph = new C_Graph();
   mygraph->directed = false; //normalement à true mais les instances ont toutes des couts symmétriques
  if(activateprint) cout << "*** STARTS READING FILE : " << filename << std::endl;

   //ifstream inFile(filename, ios::in);
    ifstream inFile;
    inFile.open(filename);
    if(inFile.fail()) {
    	cout << "ERROR FILE NOT FOUND !!!!" << endl;
    	return nullptr;
    }
    
	char oneline[MAXLINE];

	bool depotisinnodes = true;
	vector<double> depotvals;

	//////////////////////////////////////////////////////
	//start by going at the end to check which kind of DEPOT_SECTION we have
	bool readingdepotsection = false;
	while(inFile) {
		inFile.getline(oneline, sizeof(oneline));
		std::stringstream sstream(oneline);
		string word;
		sstream >> word;
		//cout << "word= "<< word << endl;
		if(strstr(word.c_str(), "DEPOT_SECTION") != nullptr) { 
			readingdepotsection = true;
		}
		else if(readingdepotsection){
			if(strcmp(word.c_str(),"-1")==0) {
				//cout << "END OF DEPOT_SECTION" << endl;
				readingdepotsection = false;
			} else {
				//cout << "line = " << oneline << endl;
				char * pch;
				pch = strtok (oneline," ");
				while (pch != NULL)
				{
					depotvals.push_back(atof(pch));
					//cout << pch << endl;
					pch = strtok (NULL, " ");
				}
				cout << "depot section data : "<< endl;
				for (int i=0; i < depotvals.size(); i++) {
					cout << i << "  -> "<<depotvals[i] << endl;
				}
				cout << "--------------" << endl;

				if(depotvals.size() == 2) depotisinnodes = false;

				readingdepotsection = false;
			}
		}
	}
	inFile.close();
	//reopen the file
	inFile.open(filename);
    if(inFile.fail()) {
    	cout << "ERROR FILE NOT FOUND !!!!" << endl;
    	return nullptr;
    }
    if(depotisinnodes) cout << "DETECTED FILE WHERE DEPOT IS IN THE NODES" << endl;
    else cout << "DETECTED FILE WHERE DEPOT IS OUTSIDE THE NODES" << endl;
	//////////////////////////////////////////////////////
	
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
			//cout << word << endl;
		}

		if(strstr(word.c_str(), "DIMENSION") != nullptr) { // checks if S1 contains S2
			
			while(sstream.peek() == ' ') { sstream.ignore(); } // ignore spaces
			if(sstream.peek() == ':') sstream >> word; // and pass ':' if there is one

		    sstream >> (mygraph->nb_nodes);//read the first int value in the stream
		    if(activateprint) cout <<  mygraph->nb_nodes << " noeuds" << endl; 
		    int n = mygraph->nb_nodes ; 
		    if(mygraph->directed == true){
			    mygraph->nb_links = n * (n - 1) ; // pas /2 comme c'est des arcs	   
			    if(activateprint) cout <<  mygraph->nb_links << " arcs" << endl; 
			}
		    else {
			    mygraph->nb_links = n * (n - 1) /2 ;  
			    if(activateprint) cout <<  mygraph->nb_links << " aretes" << endl; 
		    }

		    mygraph->V_nodes.resize(mygraph->nb_nodes);
			mygraph->V_links.resize(mygraph->nb_links);

			for (int i=0 ; i < mygraph->nb_nodes ;i++){ 
		      mygraph->V_nodes[i].num = i;				
		      mygraph->V_nodes[i].L_adjLinks.clear();
		      mygraph->V_nodes[i].weight=1;					// pas de poids sur les noeuds pour nous
		    }
		    
		}
		else if(strstr(word.c_str(), "CAPACITY") != nullptr) { // checks if S1 contains S2
			while(sstream.peek() == ' ') { sstream.ignore(); } // ignore spaces
			if(sstream.peek() == ':') sstream >> word; // and pass ':' if there is one

		    sstream >> mygraph->VRP_capacity; // int value  
		    if(activateprint) cout <<  mygraph->VRP_capacity << " de capacité par véhicule" << endl;
		}
		else if(strstr(word.c_str(), "NODE_COORD_SECTION") != nullptr) { // checks if S1 contains S2
			reading_nodecoordsection = true;
			if(activateprint) cout << "NODE_COORD_SECTION" << endl;
			if(depotisinnodes==false) {
				// we load the depot before reading the remaining nodes
				mygraph->get_node_by_id_startat0(0)->x = depotvals[0];
				mygraph->get_node_by_id_startat0(0)->y = depotvals[1];
			}
		}
		else if (reading_nodecoordsection == true) {
			int id = 0;
			sstream >> id;	//id
			if(depotisinnodes) id=id-1;
			//cout << "reading node id " << id << endl;
			if(id < 0) { // problem, no id should be  < 0
				reading_nodecoordsection = false;
				if(activateprint) cout << "DEMAND_SECTION" << endl;
				reading_demandsection = true; //pass to next section
			}
			else {
				sstream >> mygraph->get_node_by_id_startat0(id)->x;	//x
				sstream >> mygraph->get_node_by_id_startat0(id)->y;	//y

				if(activateprint) cout << "lecture coordonnée : noeud " << id << " at (" << mygraph->get_node_by_id_startat0(id)->x << ", " << mygraph->get_node_by_id_startat0(id)->y << ")" << endl;

				if (id == mygraph->nb_nodes-1) {
					reading_nodecoordsection = false;	//so that we can read the next section title after
					//OK VÉRIFIE
				}
			}
		}
		
		else if(strstr(word.c_str(), "DEMAND_SECTION") != nullptr) { // checks if S1 contains S2
			reading_demandsection = true;
			if(activateprint) cout << "DEMAND_SECTION" << endl;
			if(depotisinnodes==false) {
				// we load the depot before reading the remaining nodes
				mygraph->get_node_by_id_startat0(0)->VRP_demand = 0;
			}
		}
		else if (reading_demandsection == true){
			int id = 0;
			sstream >> id;
			if(depotisinnodes) id=id-1;
			if(id < 0) { // problem, no id should be < 0
				reading_demandsection = false;
			}
			else{
				sstream >> mygraph->get_node_by_id_startat0(id)->VRP_demand;

				if(activateprint) cout << "lecture demande : noeud " << id << " demands " << mygraph->get_node_by_id_startat0(id)->VRP_demand << endl;

				if (id == mygraph->nb_nodes-1) {
					reading_demandsection = false;	//so that we can read the next section title after
					//OK VÉRIFIE
				}
			}
		}
		else {
			//do nothing
		}       
	}
	if(activateprint) cout << "END OF FILE READING" << endl;

	C_link *a;
	int k = 0;
	C_node *noeudi, *noeudj;

	bool print_links = false; // make this true to print all the links as they are being added
	for (int i=0 ; i < mygraph->nb_nodes ; i++){
		if(print_links) cout << "adding links : ";
		for(int j=i+1 ; j < mygraph->nb_nodes ; j++) { // double boucle pour graphe complet
			a = new C_link;
			a->num = k;
			noeudi = &(mygraph->V_nodes[i]); // V_nodes[i] because we use int i = 0 to start
			noeudj = &(mygraph->V_nodes[j]);
			a->v1 = noeudi->num ; // OK VERIF
			a->v2 = noeudj->num ; // OK VERIF
			a->length = EUC_2D(noeudi->x, noeudi->y, noeudj->x, noeudj->y); // why not ? it's cheap
			mygraph->V_nodes[i].L_adjLinks.push_back(a);
			mygraph->V_nodes[j].L_adjLinks.push_back(a); //si on met dans les sens  = version non dirigée Undirected, sinon changer, voir code dans C_graph.cpp
			mygraph->V_links[k] = a;
			k++;

			if(print_links) cout << noeudi->num << "-" << noeudj->num << "(size " << a->length << ")\t";
		}
		if(print_links) cout << endl;
	}

	if(activateprint) cout << k <<" LINKS CONSTRUITS" << endl;

	if(mygraph->directed == false)	
		mygraph->construct_Undirected_Lemon_Graph();
	else 
		mygraph->construct_Directed_Lemon_Graph();

	if(activateprint) cout << "LEMON OK" << endl;

	inFile.close();

	if(activateprint) cout << "*** FILE CLOSED" << endl;
	cout << "*** FILE SUCCESSFULLY LOADED INTO C_Graph" << endl;

	//test C_node->getDistanceFrom()
	//cout << "test getdistance from node_" << mygraph->V_nodes[0].num << " and node_2" << endl;
	//cout << "\t\t\t => found distance = " << mygraph->V_nodes[0].getDistanceFrom(2) << endl;
	// OK CA MARCHE

	return mygraph;
}


//calcule la distance euclidienne dans un monde en 2D
float EUC_2D (int x1, int y1, int x2, int y2){
	return round( sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))); //this is faster than pow(a,b)
}
#endif
