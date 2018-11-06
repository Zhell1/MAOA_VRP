# MAOA_VRP

On est censé avoir des arcs (graphe orienté) mais étant donné que toutes les instances fournissent juste des noeuds avec des distances euclidiennes entre eux, les coûts sont toujours symétriques, par conséquent on peut se ramener au cas d'un graphe non-orienté.

j'ai mis directement la bonne distance dans les arêtes c'est  à dire dans: C_link->length;

Attention à la numérotation dans les fichiers les ID des noeuds commencent à 1 mais dans l'objet Graph les listes commencent à 0
pour simplifier j'ai créé ces fonctions qui utilisent la numérotation des fichiers (à partir de 1):

float C_link::getDistance()
int C_link::getidv1()
int C_link::getidv2()

float C_node::getDistanceFrom(int idj)
