#!/bin/bash

# Runs all the *_experiment's in the current directory
# Takes an optional argument that is the timeout

GREEN='\033[0;32m'
NC='\033[0m' # No Color

for experiment in *_experiment
do
    # If the experiment exists
    if [ ! -f "./$experiment" ]
    then
    echo "Experiment does not exists ($experiment)"
    exit 0
    fi
    # Run the experiment
    echo -e "${GREEN}>>> Running $experiment <<<${NC}"
    if [ $# -eq 1 ]
    then
        timeout --foreground $1 ./$experiment
    else
        ./$experiment
    fi
    echo ""
done