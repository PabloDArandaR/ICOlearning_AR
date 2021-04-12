#!/bin/sh

file_name=evolution_both.csv

current_time=$(date "+%d-%m_%H.%M")

new_file_name=evolution_$current_time.csv

cp $file_name $new_file_name

echo "New file name: $new_file_name"

scp $new_file_name $1@$2:/home/$1/Desktop/MY_THESIS/Experiments/ICO_roll/