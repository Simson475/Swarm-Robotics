#!/bin/bash

# $1 is the name of the experiment
if [ ! $# -eq 1 ]
then
echo "Missing arguments. Expected format is:"
echo "./run_experiment.sh EXPERIMENT_NAME"
exit 0
fi

# If the experiment exists
if [ -f "./$1" ]
then
echo "Experiment does not exists"
exit 0
fi

./$1 
