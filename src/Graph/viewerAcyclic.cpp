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
    cerr<<"    Graph instance having the GRA format (XX.gra)"<<endl;
    cerr<<"    Solution of the acyclic induced subgraph problem (XX.acycl)"<<endl;
    return 1;
  }


  name=argv[1];
  nameext=name+".gra";
  nameext2=name+".acycl";

  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" "<<" not found"<<endl;
    return 1;
  }

  C_Graph G;

  G.read_directed_GRA(fic);

  fic.close();

 ifstream fic2(nameext2.c_str());

  if (fic2==NULL){
    cerr<<"file "<<nameext2<<" not found"<<endl;
    return 1;
  }
  
  vector<int> sol;
  sol.resize(G.nb_nodes);
  
  for (i=0;i<G.nb_nodes;i++)
    fic2>>sol[i];
  
  fic2.close();

  
  G.write_dot_directed_G_induced(name.c_str(),sol);


  return 0;
}
