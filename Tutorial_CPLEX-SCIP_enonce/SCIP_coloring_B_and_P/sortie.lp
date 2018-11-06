\ENCODING=ISO-8859-1
\Problem name: IloCplex

Maximize
 obj: x_1 + x_3 + x_4
Subject To
 CEdge_0_1: x_0 + x_1 <= 1
 CEdge_0_2: x_0 + x_2 <= 1
 CEdge_1_3: x_1 + x_3 <= 1
 CEdge_1_4: x_1 + x_4 <= 1
 CEdge_2_3: x_2 + x_3 <= 1
 CEdge_3_4: x_3 + x_4 <= 1
Bounds
 0 <= x_0 <= 1
 0 <= x_1 <= 1
 0 <= x_2 <= 1
 0 <= x_3 <= 1
 0 <= x_4 <= 1
Binaries
 x_0  x_1  x_2  x_3  x_4 
End
