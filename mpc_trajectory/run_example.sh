#!/bin/bash
rm mpcc_log.csv > /dev/null 2>&1
python3 examples/run_example.py
python3 examples/utils/plot_results.py 