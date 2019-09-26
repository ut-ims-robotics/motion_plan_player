#!/bin/bash
echo -e `seq -s'\n' 1 100` | awk -f generate_data.awk > dataset.csv
