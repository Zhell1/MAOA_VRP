#!/bin/bash

echo "" > time.dat

c=0;
for i in *.result
do
echo -n "$c $i " >> time.dat
cat $i >> time.dat
((++c))
done

cat time.dat
