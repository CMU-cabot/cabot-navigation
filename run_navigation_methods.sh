#!/bin/bash

# Ensure we are in the directory where the script is located
cd "$(dirname "$0")"

# Check if .env exists
if [ ! -f .env ]; then
    echo "Error: .env file not found!"
    exit 1
fi

# Backup .env file
cp .env .env.bak

# Function to restore .env on exit
cleanup() {
    echo "Restoring original .env file..."
    mv .env.bak .env
}
trap cleanup EXIT INT TERM

# Iterate through the navigation methods 1, 2, 3
for method in 1 2 3; do
    echo "=========================================="
    echo "Running with CABOT_NAVIGATION_METHOD=$method"
    echo "=========================================="

    # Update CABOT_NAVIGATION_METHOD in .env
    # Using sed to replace the line in place.
    # Assuming CABOT_NAVIGATION_METHOD is already in the file.
    if grep -q "^CABOT_NAVIGATION_METHOD=" .env; then
        sed -i "s/^CABOT_NAVIGATION_METHOD=.*/CABOT_NAVIGATION_METHOD=$method/" .env
    else
        echo "CABOT_NAVIGATION_METHOD=$method" >> .env
    fi

    # Run the launch script
    ./launch.sh -s -t -d -R
    
    if [ $? -ne 0 ]; then
        echo "Warning: ./launch.sh exited with error for method $method"
    fi
    
    echo "Finished run for method $method"
    echo ""
    
    # Optional: wait a bit between runs to ensure cleanup of previous run
    sleep 5
done

echo "All runs completed."
