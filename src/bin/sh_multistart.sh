#!/bin/bash

for i in  ../Instances/CVRP_Projet/*.vrp
do
echo ""
echo " starting instance $i ..."
echo ""
echo `./CVRP_Projet $i`
done

cd "../Instances/CVRP_Projet"
echo `./sh_aggregate_times.sh`
