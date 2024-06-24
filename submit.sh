#!/bin/bash

mkdir -p slurmfiles
# Source the shared variables
source variables.sh

# Calculate total number of jobs
total_jobs=$(( ${#size_array[@]} * ${#delta_array[@]} * seed_count ))

# Create the dynamic_submit.sbatch file
cat > dynamic_submit.sbatch <<EOL
#!/bin/bash

#SBATCH --time=3-00:00:00         # Time limit (2 days)
#SBATCH --partition=compute      # Partition name
#SBATCH --array=1-${total_jobs}  # Array job
#SBATCH --mem-per-cpu=2G         # Memory per CPU
#SBATCH --output=slurmfiles/job_%A_%a.out   # Output file
#SBATCH --error=slurmfiles/job_%A_%a.err    # Error file
#SBATCH --mail-user=geet.rakala@oist.jp  # Email address for notifications
#SBATCH --mail-type=FAIL,END     # Types of email notifications

# Source the shared variables
source variables.sh

# Function to calculate parameters and execute run.sh with them
run_experiment() {
  local task_id=\$1

  # Calculate indices for arrays based on task_id
  local index_size=\$(( (task_id-1) / (seed_count * \${#delta_array[@]}) ))
  local index_delta=\$(( (task_id-1) / seed_count % \${#delta_array[@]} ))
  local index_seed=\$(( (task_id-1) % seed_count ))

  local size=\${size_array[\$index_size]}
  local delta=\${delta_array[\$index_delta]}
  local seed=\$((index_seed + 1))

  # Run the experiment
  ./run.sh \$seed \$size \$delta
}

# Execute the function with the array task ID
run_experiment \$SLURM_ARRAY_TASK_ID

EOL

# Submit the job
sbatch dynamic_submit.sbatch
#rm dynamic_submit.sbatch
