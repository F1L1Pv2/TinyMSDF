#/bin/sh

set -xe

clang++ -std=c++17 -g main.cpp -o main.exe 3dmath.cpp -fopenmp -Wno-unused-value
./main.exe