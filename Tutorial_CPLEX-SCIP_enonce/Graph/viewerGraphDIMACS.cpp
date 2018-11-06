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
    cerr<<"usage: "<<argv[0]<<" <DIMACS file name witout extension>"<<endl;
    cerr<<"    Graph instance having the DIMACS format (*.dim)"<<endl;
    return 1;
  }

  name=argv[1];
  nameext=name+".dim";


  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" "<<" not found"<<endl;
    return 1;
  }

  C_Graph G;

  G.read_undirected_DIMACS(fic);

  fic.close();

  G.write_dot_G(name.c_str());


  return 0;
}
