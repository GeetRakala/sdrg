#!/bin/bash

# Using GNU Parallel to run the experiments
# seed::size::theta
parallel ./run.sh ::: {1..500} ::: 10 ::: -1.0 -0.5 0 0.5 1.0
