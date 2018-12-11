# MAOA_VRP

TODO:

définir le problème de sac-à-dos relâché :

- contrainte de sac à dos de la capacité "Q" de tous les véhicules
- minimiser le nombre de boites (= "m") à utiliser pour répartir les clients(= objets à mettre dans les boites)

- on obtient soit une solution réalisable (= en "m" tournées ou moins), soit une solution utilisant plus de "m" tournées mais chacune réalisable, dans ce cas utiliser une méthode itérative pour l'améliorer (voir sujet pdf)

_______________________________

Remarque:


On est censé avoir des arcs (graphe orienté) mais étant donné que toutes les instances fournissent juste des noeuds avec des distances euclidiennes entre eux, les coûts sont toujours symétriques, par conséquent on peut se ramener au cas d'un graphe non-orienté.

j'ai mis directement la bonne distance dans les arêtes c'est  à dire dans: C_link->length;

Attention à la numérotation dans les fichiers les ID des noeuds commencent à 1 mais dans l'objet Graph les listes commencent à 0. Pour simplifier j'ai créé ces fonctions qui utilisent la numérotation des fichiers (à partir de 1):

- float C_link::getDistance()
- int C_link::getidv1()
- int C_link::getidv2()
- float C_node::getDistanceFrom(int idj)
