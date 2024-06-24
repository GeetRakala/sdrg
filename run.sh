#!/bin/bash

# Function to update the config file and run the program
# $1: seed
# $2: lattice_size
# $3: delta

mkdir -p "csvfiles"
# Create a unique directory for this set of parameters
dir_name="run_seed_${1}_lattice_${2}_delta_${3}"
mkdir -p "${dir_name}"

# Copy config.txt and Makefile into the new directory
cp config.txt Makefile do_sdrg "${dir_name}/"

# Go into the directory
cd "${dir_name}"

# Update the parameters in the copied config.txt
sed -i "s/^seed=.*/seed=$1/" config.txt
sed -i "s/^lattice_size=.*/lattice_size=$2/" config.txt
sed -i "s/^delta=.*/delta=$3/" config.txt

# Run the program and capture the output
#output_filename="output.txt"
#make run > "${output_filename}"
make run
mv ./csvfiles/*_results.csv ../csvfiles/
mv ./csvfiles/*_statistics.csv ../csvfiles/
cat ./csvfiles/metadata.csv >> ../csvfiles/metadata.csv
# Optionally, go back to the original directory
cd -
rm -r "${dir_name}"
