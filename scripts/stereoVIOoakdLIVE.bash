#!/bin/bash
###################################################################
# Fill the variables below

# Specify path of the Oakd dataset.
# The path can be absolute, or relative to this file location.
DATASET_PATH="/home/smartslab/uuv_ws/src/BlueROV-and-Simulations/Kimera_stuff/oak-d-data"

# Specify: 0 to run on Oakd data, 1 to run on Kitti (not supported), 2 Oak-d, 3 OakdLIVE
DATASET_TYPE=3

# Specify: 1 to enable the LoopClosureDetector, 0 to not.
USE_LCD=0

# Specify: 1 to enable logging of output files, 0 to not.
LOG_OUTPUT=1
###################################################################

###################################################################
# Other PATHS
# All paths can be absolute or relative to this file location.

# Build path: specify where the executable for Kimera is.
BUILD_PATH="/home/smartslab/kimera_ws/build/kimera_vio"

# Params path: specify where the parameters for Kimera are.
PARAMS_PATH="../params/oakd"
# PARAMS_PATH="../params/OakdMono"  # use this for monocular-mode (left cam only)

# Vocabulary path: specify where the vocabulary for loop closure is.
VOCABULARY_PATH="../vocabulary"

# Output path: specify where the output logs will be written.
# (only used if LOG_OUTPUT is enabled)
OUTPUT_PATH="../output_logs"
###################################################################

# Parse Options.
if [ $# -eq 0 ]; then
  # If there is no options tell user what are the values we are using.
  echo "Using dataset at path: $DATASET_PATH"
else
  # Parse all the options.
  while [ -n "$1" ]; do # while loop starts
      case "$1" in
        # Option -p, provides path to dataset.
      -p) DATASET_PATH=$2
          echo "Using dataset at path: $DATASET_PATH"
          shift ;;
        # Option -d, set dataset type
      -d) DATASET_TYPE=$2
          echo "Using dataset type: $DATASET_TYPE"
          echo "0 is for Oakd and 1 is for kitti"
          shift ;;
      -lcd) USE_LCD=1
           echo "Run VIO with LoopClosureDetector!" ;;
      -log) LOG_OUTPUT=1
           echo "Logging output!";;
      --)
          shift # The double dash which separates options from parameters
          break
          ;; # Exit the loop using break command
      *) echo "Option $1 not recognized" ;;
      esac
      shift
  done
fi

# Change directory to parent path, in order to make this script
# independent of where we call it from.
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

echo """ Launching:

            ██╗  ██╗██╗███╗   ███╗███████╗██████╗  █████╗
            ██║ ██╔╝██║████╗ ████║██╔════╝██╔══██╗██╔══██╗
            █████╔╝ ██║██╔████╔██║█████╗  ██████╔╝███████║
            ██╔═██╗ ██║██║╚██╔╝██║██╔══╝  ██╔══██╗██╔══██║
            ██║  ██╗██║██║ ╚═╝ ██║███████╗██║  ██║██║  ██║
            ╚═╝  ╚═╝╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝

 """

# Execute stereoVIOoakdLIVE with given flags.
# The flag --help will provide you with information about what each flag
# does.
$BUILD_PATH/stereoVIOoakdLIVE \
  --dataset_type="$DATASET_TYPE" \
  --live_dataset_path="$DATASET_PATH" \
  --initial_tt=50 \
  --final_tt=10000 \
  --params_folder_path="$PARAMS_PATH" \
  --use_lcd="$USE_LCD" \
  --vocabulary_path="$VOCABULARY_PATH/ORBvoc.yml" \
  --flagfile="$PARAMS_PATH/flags/stereoVIOEuroc.flags" \
  --flagfile="$PARAMS_PATH/flags/Mesher.flags" \
  --flagfile="$PARAMS_PATH/flags/VioBackend.flags" \
  --flagfile="$PARAMS_PATH/flags/RegularVioBackend.flags" \
  --flagfile="$PARAMS_PATH/flags/Visualizer3D.flags" \
  --logtostderr=1 \
  --colorlogtostderr=1 \
  --log_prefix=1 \
  --v=0 \
  --vmodule=Pipeline*=00 \
  --log_output="$LOG_OUTPUT" \
  --log_oakd_gt_data=0 \
  --save_frontend_images=1 \
  --visualize_frontend_images=1 \
  --output_path="$OUTPUT_PATH"

# If in debug mode, you can run gdb to trace problems.
#export PARAMS_PATH=../params/Euroc
#export DATASET_PATH=/home/tonirv/datasets/EuRoC/V1_01_easy
#gdb --args ../build/stereoVIOEuroc --flagfile="$PARAMS_PATH/flags/stereoVIOEuroc.flags" --flagfile="$PARAMS_PATH/flags/Mesher.flags" --flagfile="$PARAMS_PATH/flags/VioBackend.flags" --flagfile="$PARAMS_PATH/flags/RegularVioBackend.flags" --flagfile="$PARAMS_PATH/flags/Visualizer3D.flags" --logtostderr=1 --colorlogtostderr=1 --log_prefix=0 --dataset_path="$DATASET_PATH" --params_folder_path="$PARAMS_PATH" --initial_t=50 --final_t=2000 --vocabulary_path="../vocabulary/ORBvoc.yml" --use_lcd="0" --v=0 --vmodule=VioBackend=0 --dataset_type="0" --log_output="1" --output_path="../output_logs/"
