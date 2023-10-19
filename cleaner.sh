#!/bin/bash


# Delete all output folders and files except output.csv
find . -type d -name 'run_seed_*' -exec rm -rf {} +
