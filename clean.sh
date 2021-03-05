#!/bin/bash 

echo "Are you sure to clean deploy_temp ?"
read line
if [[ "$line" == "y" ]]; then
    rm -rf "./deploy_temp"
fi

mkdir "./deploy_temp"


echo "Are you sure to clean video_temp ?"
read line
if [[ "$line" == "y" ]]; then
    rm -rf "./video_temp"
fi

mkdir "./video_temp"