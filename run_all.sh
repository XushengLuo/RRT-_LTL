#!/bin/bash

for ((n=1;n<8;n++))
#for n in 0.25 0.20 0.15 0.1 0.05
do
    echo "--# --------------------m = ${n}------------------------"
    for ((k=1;k<6;k++))
    do
       echo "--# --------------------task = ${k}------------------------"
       for ((m=0;m<5;m++))
       do
        /usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/GitHub/RRT*_LTL/BiasOptPlan4MulR.py ${n} ${k}
       done
   done
done
# for ((n=3;n<4;n++))
# do
#     echo "---------------------------n=${n}-------------------------"
#     for h in 10 15 20
#     do
#         echo "---------------------h=${h}-----------------------------"
#         /usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/GitHub/RRT*_LTL/SMT4MulR_2.py ${h}
#     done
# done
