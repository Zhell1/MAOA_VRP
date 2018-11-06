#include "Graph.h"

using namespace std;

#define epsilon 0.00001

/****************************  C_link  *******************************/

int C_link::return_other_extrem(int v){
	return (v==v1?v2:v1);
}




/***************************  C_node  *****************************/

bool C_node::test_neighbour(int j){
  list<C_link*>::iterator it;
  for(it=L_adjLinks.begin() ; it !=L_adjLinks.end() ; it++){
    if((*it)->return_other_extrem(num) == j)
      return true;
  }
  return false;
}

bool C_node::test_successor(int j){
  list<C_link*>::iterator it;
  for(it=L_adjLinks.begin() ; it !=L_adjLinks.end() ; it++){
    if((*it)->return_other_extrem(num) == j)
      return true;
  }
  return false;
}

/**************************  C_Graph  ******************************/


void C_Graph::read_undirected_DIMACS(istream & fic){
  if (!fic){
    cout<<"File Error"<<endl;
  }else{
    int k,i,j;
    string m1,m2;
    list<C_link>::iterator it;
    C_link *a;

    fic>>m1;
    fic>>m2;
		
    // Jump the file description and go to the data
    while (((m1!="p")&&(m2!="edge"))||((m1!="p")&&(m2!="col"))){
      m1=m2;
      fic>>m2;
    }

    directed=false;
    
    fic>>nb_nodes;
    fic>>nb_links;
		
    V_nodes.resize(nb_nodes);
    V_links.resize(nb_links);

    for (i=0;i<nb_nodes;i++){
      V_nodes[i].num = i;
      V_nodes[i].L_adjLinks.clear();
      V_nodes[i].weight=1;
    }

    for (k=0;k<nb_links;k++){
      fic>>m1;
      fic>>i;
      fic>>j;

      a=new C_link;
      a->num=k;
      a->v1=min(i-1,j-1);
      a->v2=max(i-1,j-1);
      a->weight=0;
      V_nodes[i-1].L_adjLinks.push_back(a);
      V_nodes[j-1].L_adjLinks.push_back(a);
      V_links[k] = a;
    }
	
  }

}


void C_Graph::write_dot_G(string InstanceName){
  list<C_link>::iterator it;
  int i,k;

  ostringstream FileName; 
  FileName.str("");
  FileName <<InstanceName.c_str() << "_G.dot";

  ofstream fic(FileName.str().c_str());

  if (!directed) {
  
  fic<<"graph G {"<<endl;

  for(i=0 ; i<nb_nodes ; i++)
      fic<<"  "<<V_nodes[i].num<<"[shape = octagon]"<<endl;

  for(k=0 ; k<nb_links ; k++)
      fic<<"  \""<<V_links[k]->v1<<"\"--\""<<V_links[k]->v2<<"\";"<<endl;
  
  fic<<"}"<<endl;

  }
  else{


  }

  

  fic.close();

  ostringstream commande; 
  commande.str("");
  commande<<"dot -Tpdf -o "<<InstanceName.c_str() << "_G.pdf "<< FileName.str().c_str()<<endl;
  cout<<commande.str().c_str();
  if(system(commande.str().c_str())){cout<<"PDF generated successfully"<<endl;}
  return;
}



void C_Graph::write_dot_G_stableset(string InstanceName, vector<int>& stable){
  int i,k;
  ostringstream FileName; 
  FileName.str("");
  FileName <<InstanceName.c_str() << "_G_stable.dot";

  ofstream fic(FileName.str().c_str());
  
  fic<<"graph G {"<<endl;
  
  for(i=0 ; i<nb_nodes ; i++){
    if (stable[i]>1-epsilon)
      fic<<"  "<<V_nodes[i].num<<"[shape = octagon]"<<endl;
    else
      if (stable[i]<epsilon)
	fic<<"  "<<V_nodes[i].num<<"[shape = octagon, color=white]"<<endl;
  }
  
  for(k=0 ; k<nb_links ; k++){
    if ((stable[V_links[k]->v1]>1-epsilon)&&(stable[V_links[k]->v2]>1-epsilon))
      fic<<"  \""<<V_links[k]->v1<<"\"--\""<<V_links[k]->v2<<"\";"<<endl;
    else
      fic<<"  \""<<V_links[k]->v1<<"\"--\""<<V_links[k]->v2<<"\" [color=white];"<<endl;
  }
  
  fic<<"}"<<endl;
		
  fic.close();

  ostringstream commande; 
  commande.str("");
  commande<<"dot -Tpdf -o "<<InstanceName.c_str() << "_G_stable.pdf "<< FileName.str().c_str()<<endl;
  cout<<commande.str().c_str();
  if (system(commande.str().c_str())){cout<<"PDF generated successfully"<<endl;}


}





void C_Graph::write_dot_G_color(string InstanceName, vector<int>& coloring){
  int i,k;
  ostringstream FileName; 
  FileName.str("");
  FileName <<InstanceName.c_str() << "_G_color.dot";

  vector <string> colors;
  colors.push_back("green");
  colors.push_back("blue");
  colors.push_back("red");
  colors.push_back("cyan");
  colors.push_back("yellow");
  colors.push_back("magenta");
  colors.push_back("darkorchid");
  colors.push_back("darkorange");
  colors.push_back("deeppink");
  colors.push_back("forestgreen3");
  colors.push_back("indigo");
  colors.push_back("midnightblue");
  colors.push_back("violetred");

  int chi=0;
  for (i=0;i<nb_nodes;i++)
    if (chi<coloring[i]) chi=coloring[i];

  
  if(chi >= (int)colors.size()){
    cout<<"We only have 13 colors and this solutions needs "<<chi<<" colors... some nodes will have wrong colors!"<<endl;
  }
  
  ofstream fic(FileName.str().c_str());
  fic<<"graph G {"<<endl;
  
  for(i=0 ; i<nb_nodes ; i++){
    fic<<"  "<<V_nodes[i].num<<"[shape = octagon, style = filled , fillcolor = "<<colors[(coloring[V_nodes[i].num]) % colors.size()]<<" ]"<<endl;
  }
  
  for(k=0 ; k<nb_links ; k++){
      fic<<"  \""<<V_links[k]->v1<<"\"--\""<<V_links[k]->v2<<"\";"<<endl;
  }
  
  fic<<"}"<<endl;
		
  fic.close();

  ostringstream commande; 
  commande.str("");
  commande<<"dot -Tpdf -o "<<InstanceName.c_str() << "_G_color.pdf "<< FileName.str().c_str()<<endl;
  cout<<commande.str().c_str();
  if (system(commande.str().c_str())){cout<<"PDF generated successfully"<<endl;}
  

}
