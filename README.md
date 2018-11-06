# MAOA_VRP

On est censé avoir des arcs (graphe orienté) mais étant donné que toutes les instances fournissent juste des noeuds avec des distances euclidiennes entre eux, les coûts sont toujours symétriques, par conséquent on peut se ramener au cas d'un graphe non-orienté.

est-ce que c'est réellement utile de créer toutes les arêtes dans l'objet C_Graph sachant que le graphe est complet ? nos algos peuvent peut-être le gérer de leur coté

j'ai mis directement la bonne distance dans les arêtes c'est  à dire dans: C_link->length;

TODO

dans c_graph 

link
getDistance();

node
float getDistanceFrom(int j); 
