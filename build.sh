#!/usr/bin/env bash

set -e # exit on errors

cmake -S . -B build
cd build
make -j$(numcpus)

# todo: make this dynamic
# cp main.uf2 /run/media/derock/RPI-RP2/
#
sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program main.elf verify reset exit"
# minicom -b 115200 -o -D /dev/ttyACM0
