#!/bin/bash

#for r#  in 0.15 0.12 0.1 0.07 0.04 0.02
# do
#    echo "----------------------r = ${r}------------------------"
#    for ((n=0;n<20;n++))
#    do
#        /usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRGLTL/OptPlan4MulR.py ${r}
#    done
# done
for ((n=2;n<3;n++))
#for n in 0 .20 # 0.15 0.1 0.05
do
    echo "--# --------------------n = ${n}------------------------"
    for s in 0.25 0.20 0.15 0.10 0.05
    do
        echo "--# --------------------s = ${s}------------------------"

        for ((m=0;m<10;m++))
        do
                                                                                                                                                       # case r n_max
        #/usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRT*_LTL/OptPlan4MulR.py 1 ${n} 600
        #/usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRT*_LTL/SMT4MulR.py 2 #${n}
        #/usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRGLTL/OptPlan4MulR.py ${n}
            /usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/GitHub/RRT*_LTL/BiasOptPlan4MulR.py ${n} ${s} 100000
        done
    done
done
# for ((n=3;n<4;n++))
# do
#     echo "---------------------------n=${n}-------------------------"
#     for h in 10 15 20 25 30
#     do
#         echo "---------------------h=${h}-----------------------------"
#         /usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRT*_LTL/SMT4MulR.py ${n} ${h}
#     done
# done
