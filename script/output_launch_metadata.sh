#!/bin/bash

###############################################################################
# Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
###############################################################################

# Trap function for handling script termination
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "Launch metadata of test terminated"
    exit $1
}

# Function to display help message
function help()
{
    echo "Usage:"
    echo "  $0 [<options>]"
    echo ""
    echo "Options:"
    echo "  -h           show this help"
    echo "  -i <title>   specify the title for the launch_metadata yaml file"
    echo "  -o <output>  specify output directory"
}

# Initialize variables
title=""
output="."

# Parse command line options
while getopts "hi:o:" arg; do
    case $arg in
        h)
            help
            exit 0
            ;;
        i)
            title=$OPTARG
            ;;
        o)
            output=$OPTARG
            ;;
        *)
            help
            exit 1
            ;;
    esac
done
shift $((OPTIND-1))

# Validate output directory
if [[ ! -d "$output" ]]; then
    echo "Output directory $output does not exist. Creating it..."
    mkdir -p "$output" || { echo "Failed to create output directory. Exiting."; exit 1; }
fi

# Get the directory of the script
script_dir=$(dirname "$0")

# Change to the script directory
cd "$script_dir" || { echo "Failed to change to script directory $script_dir. Exiting."; exit 1; }

# After setting up the environment and before running the tests
commit_hash=$(git rev-parse HEAD)
diff_output=$(git diff)

# Define the log file path
log_file="$output/launch_metadata.yaml"

# Write the test information to the YAML file
{
    echo "title: $title"
    echo "commit: $commit_hash"
    echo "diff: |"
    echo "$diff_output" | sed 's/^/  /'
} >> "$log_file" || { echo "Failed to write to log file $log_file. Exiting."; exit 1; }

echo "Metadata written to $log_file"
