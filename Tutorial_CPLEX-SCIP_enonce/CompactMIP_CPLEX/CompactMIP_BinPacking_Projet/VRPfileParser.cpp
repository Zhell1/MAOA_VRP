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

C_Graph* parseVRPfile(string filename);

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
   mygraph->directed = true;

   cout << "filename =  " << filename << std::endl;

   //ifstream inFile(filename, ios::in);
    ifstream inFile;
    inFile.open(filename);
    if(inFile.fail()) {
    	cout << "ERROR FILE NOT FOUND !!!!" << endl;
    	return nullptr;
    }
    
   char oneline[MAXLINE];

   while (inFile)
   {

       inFile.getline(oneline, sizeof(oneline));
       std::stringstream sstream(oneline);
       //cout << oneline << endl;

       string word;
       sstream >> word;
	  // cout << word << endl;

       if(strcmp(word.c_str(), "DIMENSION") == 0) {
    	    sstream >> word; // ':'
    	    sstream >> mygraph->nb_nodes; // int value  
    	    cout <<  mygraph->nb_nodes << " noeuds" << endl; 
    	    int n = mygraph->nb_nodes;
    	    mygraph->nb_links = n * (n - 1) ; // pas /2 comme c'est des arcs	   
    	    cout <<  mygraph->nb_links << " arcs" << endl; 

    	    mygraph->V_nodes.resize(mygraph->nb_nodes);
  		    mygraph->V_links.resize(mygraph->nb_links);

  		    for (int i=0;i<mygraph->nb_nodes;i++){
		      mygraph->V_nodes[i].num = i;					// TODO
		      mygraph->V_nodes[i].L_adjLinks.clear();
		      mygraph->V_nodes[i].weight=1;					// TODO
		    }
       }
       else if(strcmp(word.c_str(), "CAPACITY") == 0) {
    	    sstream >> word; // ':'
    	    sstream >> mygraph->capacity_VRP; // int value  
    	    cout <<  mygraph->nb_nodes << " de capacité par véhicule" << endl; 	  
       }

       /*
	TODO ARCS
	  for (k=0;k<nb_links;k++){
	      fic>>m1;
	      fic>>i;
	      fic>>j;

	      a=new C_link;
	      a->num=k;
	      a->v1=min(i-1,j-1);
	      a->v2=max(i-1,j-1);
	      a->length=0;
	      V_nodes[i-1].L_adjLinks.push_back(a);
	      V_nodes[j-1].L_adjLinks.push_back(a);
	      V_links[k] = a;
	    }
       */
       
   }

   mygraph->construct_Directed_Lemon_Graph();

   inFile.close();

   return mygraph;
}


//calcule la distance euclidienne dans un monde en 2D
float EUC_2D (int x1, int y1, int x2, int y2){
	return sqrt((x1 - x2)^2 + (y1 - y2)^2);
}