#!/bin/bash

# Can run all experiments be using:
# ls | grep _experiment | ./run_experiment.sh

GREEN='\033[0;32m'
NC='\033[0m' # No Color

# $1 is the name of the experiment
while read experiment
do
    if [ ! $experiment = "run_experiment.sh" ]
    then
        echo -e "${GREEN}>>> Running $experiment <<<${NC}"
        # If the experiment exists
        if [ ! -f "./$experiment" ]
        then
        echo "Experiment does not exists ($experiment)"
        exit 0
        fi
        # Run the experiment
        ./$experiment
    fi
done