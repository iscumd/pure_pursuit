#!/usr/bin/env bash

cppcheck src/ --enable=performance,warning,performance,portability,information,missingInclude,style -i --error-exitcode=1 --report-progress --std=c++11 -j1 -I include/ 1 > /dev/null 2>>../error.txt
cppcheck tests/ --enable=performance,warning,performance,portability,information,missingInclude,style -i test.cpp -i --error-exitcode=1 --report-progress --std=c++11 -j1 -I include/ 1 > /dev/null 2>>../error.txt
cd ..
grep -v "Cppcheck cannot find all the include files" error.txt > error_filtered.txt
cat error_filtered.txt
rm error.txt
NUMERRS=$(wc error_filtered.txt -l | awk '{print $1}')
rm error_filtered.txt
exit $NUMERRS
