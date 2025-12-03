#!/bin/sh
mkdir scripts/scen/scen-random-large
rm scripts/scen/scen-random-large/*.scen
for s in {1..25}; do
./build/main -m scripts/map/random-32-32-20.map -N 800 -g -o scripts/scen/scen-random-large/random-32-32-20-random-large-$s.scen -s $s
./build/main -m scripts/map/random-64-64-20.map -N 2000 -g -o scripts/scen/scen-random-large/random-64-64-20-random-large-$s.scen -s $s
./build/main -m scripts/map/empty-32-32.map -N 1000 -g -o scripts/scen/scen-random-large/empty-32-32-random-large-$s.scen -s $s
./build/main -m scripts/map/empty-48-48.map -N 2000 -g -o scripts/scen/scen-random-large/empty-48-48-random-large-$s.scen -s $s
./build/main -m scripts/map/maze-32-32-4.map -N 700 -g -o scripts/scen/scen-random-large/maze-32-32-4-random-large-$s.scen -s $s
./build/main -m scripts/map/room-64-64-8.map -N 2000 -g -o scripts/scen/scen-random-large/room-64-64-8-random-large-$s.scen -s $s
./build/main -m scripts/map/warehouse-10-20-10-2-2.map -N 4000 -g -o scripts/scen/scen-random-large/warehouse-10-20-10-2-2-random-large-$s.scen -s $s
./build/main -m scripts/map/warehouse-10-20-10-2-1.map -N 4000 -g -o scripts/scen/scen-random-large/warehouse-10-20-10-2-1-random-large-$s.scen -s $s
done 