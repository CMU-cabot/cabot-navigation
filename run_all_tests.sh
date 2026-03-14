#!/bin/bash

# Configuration
TEST_DIR="/home/ai-suitcase-1/nitta_workspace/cabot/cabot-navigation/cabot_sites/cabot_sites_test/cabot_site_large_room/cabot_site_large_room"
TEST_FILES=(
    "tests_adult_100_child_0.py"
    "tests_adult_90_child_10.py"
    "tests_adult_80_child_20.py"
    "tests_adult_70_child_30.py"
    "tests_adult_60_child_40.py"
    "tests_adult_50_child_50.py"
    "tests_adult_40_child_60.py"
    "tests_adult_30_child_70.py"
    "tests_adult_20_child_80.py"
    "tests_adult_10_child_90.py"
    "tests_adult_0_child_100.py"
)

# Backup original tests.py
if [ -f "$TEST_DIR/tests.py" ]; then
    cp "$TEST_DIR/tests.py" "$TEST_DIR/tests.py.bak"
fi

for file in "${TEST_FILES[@]}"; do
    echo "Running tests from $file..."
    
    # Copy the test file to tests.py
    cp "$TEST_DIR/$file" "$TEST_DIR/tests.py"
    
    # Execute the launch script
    # -s: simulation
    # -t: test
    # -d: debug/developer? no, -d usually means docker build or directory? 
    # User said: -s -t -d -R
    # -R: record?
    
    ./launch.sh -s -t -d -R
    
    echo "Finished tests from $file"
    echo "-----------------------------------"
    
    # Optional: sleep to ensure cleanup
    sleep 5
done

# Restore original tests.py
if [ -f "$TEST_DIR/tests.py.bak" ]; then
    mv "$TEST_DIR/tests.py.bak" "$TEST_DIR/tests.py"
fi

echo "All tests completed."
