#!/bin/bash

threads=60
outer_size=245760000
selectivity=0.9
load_factor=0.5

echo "Threads:" $threads
echo "Outer Table Size:" $outer_size
echo "Selectivity:" $selectivity
echo "Load Factor:" $load_factor

make > /dev/null

probe_test() {
  echo ""
  echo "Phi Intrinsics Probe"
  rm -f phi-intrinsics-probe-results.csv phi-intrinsics-probe-log.txt
  echo "table_size; scalar; scalar_time_ns; vertical; vertical_time_ns" > phi-intrinsics-probe-results.csv

  for table_size in "4KB" "8KB" "16KB" "32KB" "64KB" "128KB" "256KB" "512KB" "1MB" "2MB" "4MB" "8MB" "16MB" "32MB" "64MB"
  do
    echo "--> Hash Table Size:" $table_size
    echo -n "    repetition:"
    for i in $(seq 20)
    do
      echo -n " "$i
      micnativeloadex ./phi_probe -a "$table_size $outer_size $selectivity $load_factor" >> phi-intrinsics-probe-results.csv 2> phi-intrinsics-probe-log.txt
    done
    echo ""
  done
}

build_test() {
  echo ""
  echo "Phi Intrinsics Build"
  rm -f phi-intrinsics-build-results.csv phi-intrinsics-build-log.txt
  echo "table_size; scalar; scalar_time_ns" > phi-intrinsics-build-results.csv

  for inner_size in 256 512 1024 2048 4096 8192 16384 32768 65536 131072 262144 524288 1048576 2097152 4194304
  do
    echo "--> Inner Size:" $inner_size
    echo -n "    repetition:"
    for i in $(seq 20)
    do
      echo -n " "$i
      micnativeloadex ./phi_build -a "$threads 24 $inner_size $inner_size $selectivity $load_factor" >> phi-intrinsics-build-results.csv 2> phi-intrinsics-build-log.txt
    done
    echo ""
  done
}

build_test
probe_test

make clean > /dev/null
