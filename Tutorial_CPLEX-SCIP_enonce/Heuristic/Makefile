#------------------------------------------------------------
#
# Please define an environnement variable PATHTUTOMIP
# equal to the XXX directory where Lemon and have been installed
# In order to these:
# edit ~/.bashrc add "export PATHTUTOMIP=XXX"
# close your xterm and star new ones
#
#------------------------------------------------------------


LEMONDIR	= $(PATHTUTOMIP)/lemon-1.3.1

LEMONLIBDIR	= $(LEMONDIR)/lib
LEMONINCDIR	= $(LEMONDIR)/include

CCLNDIRS	= -L$(LEMONLIBDIR)

CCLNFLAGS	= -lemon
    # Joke: a library AAA is given by a file libAAA.a and used here as -lAAA
    #       the lemon librairy is libemon.a which corresponds to -lemon 

CCFLAGS 	= -I$(LEMONINCDIR) 



all: Heuristic_StableSet  Heuristic_TSP



Heuristic_StableSet: Graph.o Heuristic_StableSet.o
	g++ $(CCFLAGS) $(CCLNDIRS) Heuristic_StableSet.o Graph.o -o ../bin/Heuristic_StableSet  $(CCLNFLAGS)

Heuristic_StableSet.o: Heuristic_StableSet.cpp ../Graph/Graph.h
	g++ -c $(CCFLAGS) $(CCLNDIRS) Heuristic_StableSet.cpp 

Heuristic_TSP: Graph.o Heuristic_TSP.o
	g++ $(CCFLAGS) $(CCLNDIRS) Heuristic_TSP.o Graph.o -o ../bin/Heuristic_TSP  $(CCLNFLAGS)

Heuristic_TSP.o: Heuristic_TSP.cpp ../Graph/Graph.h
	g++ -c $(CCFLAGS) $(CCLNDIRS) Heuristic_TSP.cpp 


Graph.o: ../Graph/Graph.cpp ../Graph/Graph.h
	g++ -c $(CCFLAGS) $(CCLNDIRS) ../Graph/Graph.cpp  

clean:
	rm -f *.o 
	rm -f ../bin/Heuristic_StableSet ../bin/Heuristic_TSP
