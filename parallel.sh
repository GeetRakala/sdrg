#!/bin/bash

# Using GNU Parallel to run the experiments
# seed::size::theta
parallel ./run.sh ::: {1..10} ::: 10 ::: 1.6784
