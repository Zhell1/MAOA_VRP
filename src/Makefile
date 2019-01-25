all:
	cd Graph && $(MAKE)
	cd Heuristic && $(MAKE)
	cd CompactMIP_CPLEX/CompactMIP_Acyclic  && $(MAKE)
	cd CompactMIP_CPLEX/CompactMIP_Coloring  && $(MAKE)
	cd CompactMIP_CPLEX/CompactMIP_StableSet  && $(MAKE)
	cd CompactMIP_CPLEX/CompactMIP_TSP  && $(MAKE)
	cd B_and_C_CPLEX/BC_Acyclic && $(MAKE)
	cd B_and_C_CPLEX/BC_TSP && $(MAKE)

clean:
	cd Graph && $(MAKE) clean
	cd Heuristic && $(MAKE) clean
	cd CompactMIP_CPLEX/CompactMIP_Acyclic  && $(MAKE) clean
	cd CompactMIP_CPLEX/CompactMIP_Coloring  && $(MAKE) clean
	cd CompactMIP_CPLEX/CompactMIP_StableSet  && $(MAKE) clean
	cd CompactMIP_CPLEX/CompactMIP_TSP  && $(MAKE) clean
	cd B_and_C_CPLEX/BC_Acyclic && $(MAKE) clean
	cd B_and_C_CPLEX/BC_TSP && $(MAKE) clean
	rm -fr Instances/*/*.pdf Instances/*/*.dot Instances/*/*.color Instances/*/*.tour Instances/*/*.stable Instances/*/*.acycl Instances/*/*.svg Instances/*/*.lp
