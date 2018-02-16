#!/bin/bash

rm -f cpu-ls-results.csv     \
      cpu-gs-results.csv     \
      cpu-build-results.csv  \
      cpu-probe-results.csv  \
      log.txt

run_load_store() {
	echo "Selective Load & Selective Store"
	make METHOD="_LS" DIR=$(pwd) > /dev/null
	echo "method;throughput" >> cpu-ls-results.csv
	./opencl_avx >> cpu-ls-results.csv 2>> log.txt
	make clean > /dev/null
}

run_gather_scatter() {
	echo "Gather & Scatter"
	make METHOD="_GS" DIR=$(pwd) > /dev/null
	echo "method;table_size;throughput" >> cpu-gs-results.csv
	./opencl_avx  >> cpu-gs-results.csv 2>> log.txt
	make clean > /dev/null
}

run_build() {
	echo "Build"
	make METHOD="_BUILD" DIR=$(pwd) > /dev/null
	echo "method;device;human_table_size;table_size;items;throughput;time_in_ns" >> cpu-build-results.csv
	./opencl_avx >> cpu-build-results.csv 2>> log.txt
	make clean > /dev/null
}

run_probe() {
	echo "Probe"
	make METHOD="_LP" DIR=$(pwd) > /dev/null
	echo "method;device;human_table_size;table_size;items;throughput;time_in_ns" >> cpu-probe-results.csv
	./opencl_avx >> cpu-probe-results.csv 2>> log.txt
	make clean > /dev/null
}

run_load_store
run_gather_scatter
run_build
run_probe
