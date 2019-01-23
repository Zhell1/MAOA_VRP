#------------------------------------------------------------
#
# Please define an environnement variable PATHTUTOMIP
# equal to the XXX directory where Lemon,Graphviz and Cplex have been installed
# In order to these:
# edit ~/.bashrc add "export PATHTUTOMIP=XXX"
# close your xterm and star new ones
#
#------------------------------------------------------------

#CPLEXDIR      = $(PATHTUTOMIP)/cplex-12.7.1/cplex
#CONCERTDIR    = $(PATHTUTOMIP)/cplex-12.7.1/concert
CPLEXDIR      = $(PATHTUTOMIP)/CPLEX_Studio128/cplex
CONCERTDIR    = $(PATHTUTOMIP)/CPLEX_Studio128/concert

LEMONDIR      = $(PATHTUTOMIP)/lemon-1.3.1


SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic

# ---------------------------------------------------------------------
# Compiler selection 
# ---------------------------------------------------------------------

#CCC = g++ -O0  #original
CCC = g++ -O0 -g -std=c++11  #version pour debug avec valgrind
#CCC = g++ -O3 -std=c++11 -w
#-w : remove warnings
#-g : debug symbols

# ---------------------------------------------------------------------
# Compiler options 
# ---------------------------------------------------------------------

CCOPT = -m64 -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

CPLEXBINDIR   = $(CPLEXDIR)/bin/$(BINDIST)
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
LEMONLIBDIR   = $(LEMONDIR)/lib

CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR) -L$(LEMONLIBDIR)


CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include
LEMONINCDIR   = $(LEMONDIR)/include

CCLNFLAGS = -lconcert -lilocplex -lcplex -lemon -lm -lpthread -ldl

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) -I$(LEMONINCDIR)




# ---------------------------------------------------------------------
# Your part
# ---------------------------------------------------------------------



all: CompactMIP_BinPacking_Projet

Graph.o: ../../Graph/Graph.cpp ../../Graph/Graph.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c ../../Graph/Graph.cpp -o ../../Graph/Graph.o

CompactMIP_BinPacking_Projet.o : CompactMIP_BinPacking_Projet.cpp VRPfileParser.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) CompactMIP_BinPacking_Projet.cpp

CompactMIP_BinPacking_Projet: CompactMIP_BinPacking_Projet.o   Graph.o 
	$(CCC) $(CCFLAGS) $(CCLNDIRS) CompactMIP_BinPacking_Projet.o  ../../Graph/Graph.o   -o ../../bin/CompactMIP_BinPacking_Projet $(CCLNFLAGS)


clean:
	rm -f *.o ../../bin/VRPfileParser ../../bin/CompactMIP_BinPacking_Projet



# debug

debug: VRPfileParserDebug


Graph.oDebug: ../../Graph/Graph.cpp ../../Graph/Graph.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c ../../Graph/Graph.cpp -o ../../Graph/Graph.o -g -Wall -Wextra

VRPfileParser.oDebug: VRPfileParser.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) VRPfileParser.cpp -g -Wall -Wextra

VRPfileParserDebug: VRPfileParser.o  Graph.o 
	$(CCC) $(CCFLAGS) $(CCLNDIRS) VRPfileParser.o ../../Graph/Graph.o   -o ../../bin/VRPfileParser $(CCLNFLAGS) -g -Wall -Wextra

	

############### KEEP THIS #####################
#this used VRPfileParser.cpp, has been renamed as a .h since then and the main() function in it has been commented

#VRPfileParser.o: VRPfileParser.cpp
#	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) VRPfileParser.cpp

#VRPfileParser: VRPfileParser.o  Graph.o 
#	$(CCC) $(CCFLAGS) $(CCLNDIRS) VRPfileParser.o ../../Graph/Graph.o   -o ../../bin/VRPfileParser $(CCLNFLAGS)
###############################################
