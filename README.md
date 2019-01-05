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

Attention à la numérotation dans les fichiers les ID des noeuds commencent à 1 mais dans l'objet Graph les listes commencent à 0 !!!

- float C_link::getDistance()
- int C_link::getidv1()
- int C_link::getidv2()
- float C_node::getDistanceFrom(int idj)


_______________________________
thomas:
A noter dans le rapport:

pour la métaheuristique
dans  optimize_2opt_switchRoutes() : 
  // pour rendre la fonction moins biaisée sur le fait de ne déplacer que les premiers éléments des premieres tournées
  // -> faire un shuffle des tournées aléatoire au début de la fonction
  // -> ce shuffle est plutôt bénéfique sur les grandes instances et en général, mais peu donner de moins bonnes solutions sur de petites instances si on à pas de chance
cela est du au fait que les grandes instances ont beaucoup d'itérations nécessaires pour atteindre l'optimale, donc on va shuffle de nombreuses fois et le facteur chance s'efface, alors que sur les petites instances on à peu d'itérations, donc on va shuffle peu de fois et le facteur chance est grand. Mais on peut contrer cet effet en relancer plusieurs fois la résolution sur les petites instances puisqu'elles prennent peu de temps.
(idée: on pourrait compter le nombre de shuffles qu'on fait au total et mettre un seuil experimentale constaté: si on en fait moins que ce seuil alors on relance 10x la résolution et on retourne la valeur de cout min)
  
en pratique on à pu constater que l'ajout de ce shuffle aléatoire dans la métaheuristique fait vraiment exploser la vitesse d'optimisation
par exemple avec l'instance Li_21 qui est l'une des plus grosse avec 560 noeuds, la meilleure valeur connue à ce jour est 16212.82548
sans shuffle on obtient un cout 31300 après 01h30
mais avec shuffle 1x au début de la fonction switchRoutes() :on obtient en 7 minutes seulement un cout de 28000 ! et en 10 minutes un cout de 25500, et 20800 au bout de 20 minutes, 20108 à 25min , et 19850 au bout de 30 minutes, 40 min = 19400, 45min = 19250, 50min = 19046, 1H = 18753
en rajoutant un shuffle à chaque amélioration trouvée dans la fonction switchRoute() : on atteint 28000 en 3 minutes seulement. à 7 minutes on est à 24300, au bout de 10 min on est à 22900, à 15 min on est à 20950, 20 min = 20080, 25min = 19630, 30min = 19300, 40min = 18630, 45min = 18200, 50min=17800, 1H = 17300, 1H10 = 17014
=> tracer le graphique avec les 3 courbes



1H11:

tournées : 
	 tournée #0 (cost= 570.203) (demand=620) : 38 36 35 34 33 71 70 69 109 149 189 229 269 270 230 190 152 153 113 114 115 116 117 118 119 79 
	 tournée #1 (cost= 1817.73) (demand=1080) : 42 83 123 162 202 242 282 322 362 403 364 365 366 447 488 489 529 530 531 490 410 371 411 491 532 533 413 373 333 293 253 214 175 136 137 94 93 50 49 10 9 8 7 6 5 1 
	 tournée #2 (cost= 0) (demand=0) : 
	 tournée #3 (cost= 1810.24) (demand=1130) : 47 48 88 89 90 129 130 131 132 133 174 216 217 218 219 260 261 223 224 225 226 266 306 386 426 466 506 546 547 548 549 509 510 550 551 511 552 512 433 393 353 313 273 234 194 154 76 77 78 
	 tournée #4 (cost= 2642.75) (demand=1190) : 72 151 191 231 272 312 352 392 473 513 553 554 555 516 476 396 398 438 518 558 519 559 520 440 521 481 441 401 402 442 482 522 523 483 443 484 524 485 525 526 486 446 487 449 409 369 329 288 248 209 169 166 165 164 163 122 82 
	 tournée #5 (cost= 1819.54) (demand=1170) : 44 85 86 128 168 208 249 250 251 252 212 213 254 295 296 337 338 378 377 417 457 496 536 537 497 498 538 539 540 501 421 381 341 302 262 222 182 181 180 179 178 138 98 99 100 61 21 20 19 18 17 16 15 560 
	 tournée #6 (cost= 2102.96) (demand=1200) : 43 84 124 125 126 205 245 285 325 326 327 328 289 330 290 291 331 292 294 334 374 414 494 534 535 336 257 258 259 220 221 183 143 142 141 140 139 177 134 173 172 171 211 210 170 53 13 14 62 63 25 26 37 39 
	 tournée #7 (cost= 2349.71) (demand=1200) : 41 81 121 161 160 200 201 241 240 279 238 197 196 155 112 111 110 108 187 227 267 307 305 265 264 263 303 304 344 385 424 465 545 544 463 543 542 502 422 382 342 301 340 300 299 298 297 256 255 215 176 135 95 96 97 57 56 55 54 92 91 51 52 12 11 4 3 2 
	 tournée #8 (cost= 2330.69) (demand=1200) : 32 68 67 106 147 188 228 268 308 309 349 389 428 429 469 470 430 390 350 310 351 311 271 232 193 195 156 157 158 198 237 236 235 275 315 316 276 277 318 358 359 319 280 320 361 321 281 323 324 284 283 243 203 204 244 206 246 286 287 247 207 167 127 87 46 45 
	 tournée #9 (cost= 1550.78) (demand=1200) : 40 80 120 159 199 239 278 317 357 397 437 477 517 557 556 515 514 474 434 394 354 314 274 233 192 150 148 186 185 184 144 145 146 107 66 65 64 105 104 103 102 101 60 59 58 22 23 24 27 28 29 30 31 73 74 75 
	 total_cost = 16994.6


