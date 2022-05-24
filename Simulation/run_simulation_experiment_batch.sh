#!/bin/bash

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

if [ $# -ne 5 ]
then
    echo -e "${RED}Missing arguments${NC}"
    echo "Expected format is: ./run_simulation_experiment_batch <SCENE> <BATCH_SIZE> <SEED_OFFSET> <JOB_COUNT> <TIMEOUT>"
    exit 1
fi

scene=$1
batch_size=$2
seed_offset=$3
job_count=$4
timeout=$5

SCENE_DIR='experiment'

# If the scene does not exists
if [ ! -d "${SCENE_DIR}/${scene}" ]
then
echo -e "${RED}Scene does not exists (${scene})${NC}"
exit 1
fi

# If the .argos file does not exists
# If the experiment exists
if ! ls "$SCENE_DIR/$scene/"*".argos" > /dev/null 2>&1;
then
echo -e "${RED}.argos file does not exists for ${scene}${NC}" >&2
exit 1
fi

# For simplicity assume the .argos file is called trajectory.argos
argosFile="trajectory.argos"

# If batch size is <= 0
if [ $batch_size -le 0 ]
then
echo -e "${RED}Batch size must be positive${NC}" >&2
exit 1
fi

# If seed_offset is <= 0
if [ ! $seed_offset -ge 0 ]
then
echo -e "${RED}Seed offset must be >= 0${NC}" >&2
exit 1
fi

# If job_count is <= 0
if [ ! $job_count -ge 0 ]
then
echo -e "${RED}Job count must be >= 0${NC}" >&2
exit 1
fi

############## Actual script stuff #################

# Create result dir
RESULT_DIR="SimulationExperimentResults"
if [ ! -d $RESULT_DIR ]
then
mkdir $RESULT_DIR
# Add a readme
echo "zip file names are named as <SCENE>_<JOB_COUNT>_<TIMEOUT>
Result file names should be read as <SCENE>_<JOB_COUNT>_<SEED>_<TIMEOUT>" > "$RESULT_DIR/README.md"
fi

BIN_DIR="./build/external/bin"
BIN_TO_CWD="../../.."
BIN_TO_SCENE_DIR="$BIN_TO_CWD/$SCENE_DIR"
cd $BIN_DIR

# Run the batch
echo "Running $batch_size runs in $scene with timeout of $timeout"
lineToReplace=".*<\/argos-configuration>.*"
i=0
while [ $i -ne $batch_size ]
do
echo "$i/$batch_size"
seed=$(($seed_offset + $i))
lineToReplaceWith="<experiment_settings seed=$seed jobs=$job_count timeout=$timeout\/><\/argos-configuration>"
inputFile="$BIN_TO_SCENE_DIR/$scene/$argosFile"
tempFile="Temp$argosFile"
sed "s/$lineToReplace/$lineToReplaceWith/" < $inputFile > $tempFile
mv $tempFile $inputFile
((i=i+1))
# Running in silent mode by redirecting all out streams to /dev/null
./run_a_scene.sh $scene > /dev/null 2>&1
# Clean up
lineToReplaceWith="<\/argos-configuration>"
sed "s/$lineToReplace/$lineToReplaceWith/" < $inputFile > $tempFile
mv $tempFile $inputFile
# Save the results
resultFile="${scene}_${job_count}_${seed}_${timeout}"
mv "jobProgress.txt" $resultFile
zipFile="$BIN_TO_CWD/$RESULT_DIR/${scene}_${job_count}_${timeout}"
zip -r $zipFile $resultFile
rm $resultFile
done
echo "$batch_size/$batch_size"

cd $BIN_TO_CWD
