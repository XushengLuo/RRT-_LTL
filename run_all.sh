#!/bin/bash

#for r#  in 0.15 0.12 0.1 0.07 0.04 0.02
# do
#    echo "----------------------r = ${r}------------------------"
#    for ((n=0;n<20;n++))
#    do
#        /usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRGLTL/OptPlan4MulR.py ${r}
#    done
# done
for ((n=0;n<20;n++))
do
     #echo "--# --------------------n = ${n}------------------------"
    #/usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRT*_LTL/OptPlan4MulR.py
    /usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRT*_LTL/SMT_planning.py
    #/usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/GitHub/RRT*_LTL/BiasOptPlan4MulR.py
    #/usr/local/Cellar/pypy3/6.0.0/libexec/bin/pypy3 /Users/chrislaw/Documents/GitHub/RRT*_LTL/BiasOptPlan4MulR.py
    #/usr/local/Cellar/pypy3/6.0.0/libexec/bin/pypy3 /Users/chrislaw/Documents/GitHub/RRT*_LTL/OptPlan4MulR.py
    #/usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Documents/RRT*_LTL/OptPlan4MulR.py
    #/usr/local/Cellar/pypy3/6.0.0/libexec/bin/pypy3 /Users/chrislaw/Documents/RRT*_LTL/BiasOptPlan4MulR.py ${n}
done
