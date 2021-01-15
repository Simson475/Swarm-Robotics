#!/bin/bash
#SBATCH --time=1:00:00
#SBATCH --mail-user=mk@cs.aau.dk
#SBATCH --mail-type=FAIL
#SBATCH --partition=dhabi
#SBATCH --mem=15000

mkdir -p results

for i in {1..30}
do
    sbatch ./setup_and_run_cluster.sh $i
done
