#!/bin/bash

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "Launch metadata of test terminated"
    exit $1
}

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

title=""
output=""

while getopts "hi:o:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        i)
            title=$OPTARG
            ;;
        o)
            output=$OPTARG
            ;;
    esac
done
shift $((OPTIND-1))

if [ -z "$output" ]; then
    output="."
fi

# Get the directory of the script
script_dir=$(dirname "$0")

# Change to the script directory
cd "$script_dir"

# After setting up the environment and before running the tests
commit_hash=$(git rev-parse HEAD)
diff_output=$(git diff)

log_file="$output/launch_metadata.yaml"

# Write the test information to the YAML file
echo -e "title: $title\ncommit: $commit_hash\ndiff: |\n$(echo "$diff_output" | sed 's/^/    /')" >> $log_file
