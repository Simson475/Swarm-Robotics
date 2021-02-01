#!/bin/bash

mkdir -p results_${1}

for i in {1..30}
do
    sbatch ./setup_and_run_cluster.sh $i ${1}
done
