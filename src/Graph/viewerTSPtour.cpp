#include<iostream>
#include<string>
#include<sstream>
#include<iomanip>

#include"Graph.h"

using namespace std;


int main(int argc,char**argv){

  string name, nameext, nameext2;
  int i,j;

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" <instancename without extension>"<<endl;
    cerr<<"    Graph instance having the TSP format (XX.tsp)"<<endl;
    cerr<<"    Solution (tour) of the TSP problem (XX.tour)"<<endl;
    return 1;
  }

  name=argv[1];
  nameext=name+".tsp";  
  nameext2=name+".tour";

  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" "<<" not found"<<endl;
    return 1;
  }

  C_Graph G;

  G.read_undirected_complete_TSP(fic);

  fic.close();

 ifstream fic2(nameext2.c_str());

  if (fic2==NULL){
    cerr<<"file "<<nameext2<<" not found"<<endl;
    return 1;
  }
  
  list<pair<int,int> > sol;
  sol.clear();

  int a,b;
  for (i=0;i<G.nb_nodes;i++){
    fic2>>a;
    fic2>>b;
    sol.push_back(make_pair(a,b));
  }
  
  fic2.close();

  
  G.write_SVG_tour(name.c_str(),sol);


  return 0;
}
