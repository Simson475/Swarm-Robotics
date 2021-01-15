#!/bin/bash
#SBATCH --time=1:00:00
#SBATCH --mail-user=mk@cs.aau.dk
#SBATCH --mail-type=FAIL
#SBATCH --partition=dhabi
#SBATCH --mem=1500

# Creates variable for the new folder
new_folder=exp_${1}
mkdir $new_folder

# Copies all the files needed. Needs to be updated to use sym-links
ln -s $(realpath experiment/trajectory.argos) test/trajectory.argos

ln -s $(realpath experiment/argos3-lib) ${new_folder}/argos3-lib
ln -s $(realpath experiment/bin-Linux) ${new_folder}/bin-Linux
ln -s $(realpath experiment/argos3) ${new_folder}/argos3
ln -s $(realpath experiment/libCTrajectoryLoopFunctions.so) ${new_folder}/libCTrajectoryLoopFunctions.so
ln -s $(realpath experiment/libSingleThreadUppaalBot.so) ${new_folder}/libSingleThreadUppaalBot.so
ln -s $(realpath experiment/planning_blueprint.xml) ${new_folder}/planning_blueprint.xml
ln -s $(realpath experiment/points.json) ${new_folder}/points.json
ln -s $(realpath experiment/run_a_scene.sh) ${new_folder}/run_a_scene.sh
ln -s $(realpath experiment/trajectory.argos) ${new_folder}/trajectory.argos

# Enters the folder and run the argos simulation
cd $new_folder
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./argos3-lib && ./argos3 -c ./trajectory.argos && {
    cd ..

    # Moves the results and giving it a proper name
    mv ${new_folder}/data.csv results/${new_folder}.csv

    # Remove all the data.
    rm -r $new_folder
} || {
    echo "The execution of ARGOS failed. See log-files for more details."; exit -10;
}