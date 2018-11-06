#include<iostream>
#include<string>
#include<sstream>
#include<iomanip>

#include"Graph.h"

using namespace std;


int main(int argc,char**argv){

  string name, nameext;
  int i,j;

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" <TSP file name without exension>"<<endl;
    cerr<<"    Graph instance having the TSP format (*.tsp)"<<endl;
    return 1;
  }

  name=argv[1];
  nameext=name+".tsp";


  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" "<<" not found"<<endl;
    return 1;
  }

  C_Graph G;

  G.read_undirected_complete_TSP(fic);

  fic.close();

  G.write_SVG_node_cloud(name.c_str());


  return 0;
}
