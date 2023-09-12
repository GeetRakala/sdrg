#!/bin/bash

# Function to update the config file and run the program
# $1: seed
# $2: lattice_size
# $3: theta

# Create a unique directory for this set of parameters
dir_name="run_seed_${1}_lattice_${2}_theta_${3}"
mkdir -p "${dir_name}"

# Copy config.txt and Makefile into the new directory
cp config.txt Makefile do_sdrg "${dir_name}/"

# Go into the directory
cd "${dir_name}"

# Update the parameters in the copied config.txt
sed -i "s/^seed=.*/seed=$1/" config.txt
sed -i "s/^lattice_size=.*/lattice_size=$2/" config.txt
sed -i "s/^theta=.*/theta=$3/" config.txt

# Run the program and capture the output
output_filename="output.txt"
make run > "${output_filename}"

# Optionally, go back to the original directory
cd -
