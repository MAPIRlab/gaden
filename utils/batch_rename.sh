#!/bin/bash

# changes the nonsense automatic paraview filenames to the correct format. Might not be necessary, depending on the specific version of paraview you are using
# use if the file names look like "wind0.1.csv" instead of "wind_1.csv"
# usage: ./batch_rename.sh {path/to/wind_files_folder}

cd "$(dirname "${BASH_SOURCE[0]}")"

for file in $1/*
do
    ./rename.out $file
done