#!/bin/bash

# @brief Create a CSV file to aggregate output from multiple runs. This
#        script assumes that each run's output is in a separate directory,
#        and the output file is named "output.txt".

# Create CSV file with headers
echo "Seed,Lattice_Size,Theta,EE,MI,EN" > output.csv

# Loop over each output directory to extract data
for dir in run_seed_*; do
    seed=$(echo $dir | cut -d'_' -f3)
    lattice_size=$(echo $dir | cut -d'_' -f5)
    theta=$(echo $dir | cut -d'_' -f7)

    # Define the path to the output file within the directory
    file="${dir}/output.txt"

    # Extract relevant data from the output file
    # Assumes that the output has the specific strings to grep for
    entanglement_entropy=$(grep "Entanglement Entropy = " $file | awk '{print $4}')
    mutual_information=$(grep "Mutual Information = " $file | awk '{print $4}')
    entanglement_negativity=$(grep "Entanglement Negativity = " $file | awk '{print $4}')

    # Append to CSV
    echo "$seed,$lattice_size,$theta,$entanglement_entropy,$mutual_information,$entanglement_negativity" >> output.csv
done

# Delete all output folders and files except output.csv
#find . -type d -name 'run_seed_*' -exec rm -rf {} +
