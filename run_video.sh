#!/bin/bash 


export YASK_HARDWARE="VIDEO"

echo "Video: $1"
# video_path=""
# read video_path

python3 main.py "$1"
export YASK_HARDWARE=""


