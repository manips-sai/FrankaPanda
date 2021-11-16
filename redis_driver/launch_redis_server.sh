#!/bin/bash
# Author: Adrian Piedra (2021)
#
# This program executes a given process (PROCESS_NAME) on a
# given range of CPUs (CPU_NUM_LIST). First, the desired CPUs
# are set to performance mode. Then, the process is launched
# with real-time priority and minimum niceness. Ideally, the
# CPUs assigned for this process should be isolated.
#
# This program must be launched with the bash interpreter
# or executed directly:
# -- Make the script executable with $ chmod +x SCRIPT_NAME
# -- Run the script with ./SCRIPT_NAME
#
# This program must be run as root.

# USER INPUTS
PROCESS_NAME=redis-server # process to be launched
CPU_NUM_LIST=0 # range of CPUs to use for process

# ensure the script is run with bash
if [ ! "$BASH_VERSION" ] ; then
    echo "Please do not use sh to run this script ($0), just execute it directly" 1>&2
    exit 1
fi

# ensure the script is run with root privilege
# $ sudo ./SCRIPT_NAME
if [ "$(id -u)" != "0" ]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

# set the desired CPUs to run in performance mode
sudo cpufreq-set -c ${CPU_NUM_LIST} -g "performance"

# launch the process in the background on the desired CPUs
taskset -c ${CPU_NUM_LIST} ${PROCESS_NAME} &

# remember the background process ID
MAIN_PROCESS_ID=$!

# set real-time priority for main process and all of its threads
sudo chrt -ap 99 ${MAIN_PROCESS_ID}

# remember the process ID for all threads spawned by main process
ALL_PROCESS_IDS=($(ls /proc/${MAIN_PROCESS_ID}/task))

# set minimum nice value for main process and all of its threads
sudo renice -n -39 -p ${ALL_PROCESS_IDS[@]}

# catch CTRL+C (SIGINT) from user to shut down the main process
trap "redis-cli shutdown" INT

# keep script running until the process is stopped
wait ${MAIN_PROCESS_ID}
