#!/bin/bash

for i in  ../Instances/BinPacking_Projet/*.vrp
do
echo \n starting instance $i ... \n
echo `./CompactMIP_BinPacking_Projet $i`
done

cd "../Instances/BinPacking_Projet"
echo `./sh_aggregate_times.sh`
