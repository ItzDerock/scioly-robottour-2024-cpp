#!/usr/bin/env bash

NUM_CPUS=$(cat /proc/cpuinfo | grep processor | wc -l)

set -e # exit on errors

cmake -S . -B build
cd build
make -j$NUM_CPUS


case "$1" in
    "upload")
        cp main.uf2 /run/media/derock/RPI-RP2/
        ;;
    "test")
        # Run the current stuff
        openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program main.elf verify reset exit"
        minicom -b 115200 -o -D /dev/ttyACM0
        ;;
    *)
        # Just build
       ;;
esac
