#!/bin/bash
if [ -z "$1" ]; then
        echo 'Write the name of the scene to test as an argument. For example: ./run_a_scene.sh scene2'
        exit 1
fi

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../lib/argos3 && ./argos3 -c ../../../experiment/"$1"/trajectory.argos