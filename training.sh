#!/bin/sh
./build/main -m scripts/map/random-32-32-20.map -t 60 -N 600 --objective 2 -s 1 -v 3 --traffic 8
./build/main -m scripts/map/random-64-64-20.map -t 60 -N 1000 --objective 2 -s 1 -v 3 --traffic 8
./build/main -m scripts/map/empty-32-32.map -t 60 -N 600 --objective 2 -s 1 -v 3 --traffic 8
./build/main -m scripts/map/empty-48-48.map -t 60 -N 1000 --objective 2 -s 1 -v 3 --traffic 8
./build/main -m scripts/map/maze-32-32-4.map -t 60 -N 400 --objective 2 -s 1 -v 3 --traffic 8
./build/main -m scripts/map/room-64-64-8.map -t 60 -N 1000 --objective 2 -s 1 -v 3 --traffic 8
./build/main -m scripts/map/warehouse-10-20-10-2-2.map -t 60 -N 1000 --objective 2 -s 1 -v 3 --traffic 8
./build/main -m scripts/map/warehouse-10-20-10-2-1.map -t 60 -N 1000 --objective 2 -s 1 -v 3 --traffic 8