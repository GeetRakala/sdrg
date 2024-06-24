#!/bin/bash

# Using GNU Parallel to run the experiments
# seed::size::delta
parallel ./run.sh ::: {1..10} ::: 64 ::: 0.00
