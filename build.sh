#!/usr/bin/env bash

cmake -S . -B build
cd build
make -j$(numcpus)

# todo: make this dynamic
cp main.uf2 /run/media/derock/RPI-RP2/
