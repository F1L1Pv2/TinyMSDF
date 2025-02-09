#/bin/sh

set -xe

clang++ -std=c++17 -g fontAtlas.cpp -o fontAtlas.exe 3dmath.cpp -fopenmp -Wno-unused-value
./fontAtlas.exe