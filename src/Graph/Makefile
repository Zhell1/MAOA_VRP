#------------------------------------------------------------
#
# Please define an environnement variable PATHTUTOMIP
# equal to the XXX directory where Lemon and Graphviz have been installed
# In order to these:
# edit ~/.bashrc add "export PATHTUTOMIP=XXX"
# close your xterm and star new ones
#
#------------------------------------------------------------

LEMONDIR	= $(PATHTUTOMIP)/lemon-1.3.1

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

LEMONLIBDIR	= $(LEMONDIR)/lib
CCLNDIRS  = -L$(LEMONLIBDIR)

LEMONINCDIR   = $(LEMONDIR)/include
CCFLAGS = -I$(LEMONINCDIR)

CCLNFLAGS = -lemon
   # Joke: as the librairy lemon is named libemon,
   # linking lemon librairy is just-lemon

all: viewerGraphDIMACS viewerGraphGRA viewerGraphTSP viewerStable viewerColor viewerAcyclic viewerTSPtour

 
viewerStable: Graph.o viewerStable.o
	g++  $(CCFLAGS) $(CCLNDIRS) viewerStable.o Graph.o -o ../bin/viewerStable $(CCLNFLAGS)

viewerStable.o: viewerStable.cpp
	g++ -c $(CCFLAGS) $(CCLNDIRS) viewerStable.cpp 

viewerColor: Graph.o viewerColor.o
	g++  $(CCFLAGS) $(CCLNDIRS) viewerColor.o Graph.o -o ../bin/viewerColor $(CCLNFLAGS)

viewerColor.o: viewerColor.cpp
	g++ -c $(CCFLAGS) $(CCLNDIRS) viewerColor.cpp

viewerAcyclic: Graph.o viewerAcyclic.o
	g++  $(CCFLAGS) $(CCLNDIRS) viewerAcyclic.o Graph.o -o ../bin/viewerAcyclic $(CCLNFLAGS)

viewerTSPtour: Graph.o viewerTSPtour.o
	g++  $(CCFLAGS) $(CCLNDIRS) viewerTSPtour.o Graph.o -o ../bin/viewerTSPtour $(CCLNFLAGS)

viewerTSPtour.o: viewerTSPtour.cpp
	g++ -c $(CCFLAGS) $(CCLNDIRS) viewerTSPtour.cpp 


viewerAcyclic.o: viewerAcyclic.cpp
	g++ -c $(CCFLAGS) $(CCLNDIRS) viewerAcyclic.cpp 

viewerGraphTSP: viewerGraphTSP.o Graph.o
	g++  $(CCFLAGS) $(CCLNDIRS) viewerGraphTSP.o Graph.o -o ../bin/viewerGraphTSP $(CCLNFLAGS)

viewerGraphTSP.o: viewerGraphTSP.cpp
	g++ -c $(CCFLAGS) $(CCLNDIRS) viewerGraphTSP.cpp


viewerGraphDIMACS: viewerGraphDIMACS.o Graph.o
	g++  $(CCFLAGS) $(CCLNDIRS) viewerGraphDIMACS.o Graph.o -o ../bin/viewerGraphDIMACS $(CCLNFLAGS)

viewerGraphDIMACS.o: viewerGraphDIMACS.cpp
	g++ -c $(CCFLAGS) $(CCLNDIRS) viewerGraphDIMACS.cpp

viewerGraphGRA: viewerGraphGRA.o Graph.o
	g++  $(CCFLAGS) $(CCLNDIRS) viewerGraphGRA.o Graph.o -o ../bin/viewerGraphGRA $(CCLNFLAGS)

viewerGraphGRA.o: viewerGraphGRA.cpp
	g++ -c $(CCFLAGS) $(CCLNDIRS) viewerGraphGRA.cpp 

Graph.o: Graph.cpp Graph.h
	g++ -c $(CCFLAGS) $(CCLNDIRS) Graph.cpp  

clean:
	rm -f *.o 
	rm -f ../bin/viewerGraphDIMACS ../bin/viewerGraphGRA ../bin/viewerStable ../bin/viewerColor ../bin/viewerAcyclic ../bin/viewerGraphTSP ../bin/viewerTSPtour
