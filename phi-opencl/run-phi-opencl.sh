#!/bin/bash

rm -f phi-opencl-results.csv \
      phi-opencl-log.txt

echo "Phi OpenCL Build & Probe"
make DIR=$(pwd) > /dev/null
echo "method;device;human_table_size;table_size;items;throughput;time_in_ns" >> ./phi-opencl-results.csv 2>> ./phi-opencl-log.txt
./opencl >> ./phi-opencl-results.csv 2>> phi-opencl-log.txt
make clean > /dev/null
