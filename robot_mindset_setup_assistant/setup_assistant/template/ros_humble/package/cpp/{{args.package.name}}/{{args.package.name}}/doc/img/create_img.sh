#!/bin/bash

# This script images.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

error_counter=0
scripts=(
    "$SCRIPT_DIR/urdf/create_urdf_img.sh"
)
# Initialize an empty list to accept strings
error_scripts=()

# Loop over each command and execute it
for script in "${scripts[@]}"; do
    bash "$script"
    if [ $? -ne 0 ]; then
        error_counter=$(( error_counter + 1 ))
        error_scripts+=("$script")
        echo "ERROR: $script"
    fi
done

if [ $error_counter -eq 0 ]; then
    echo -e "\033[0;32mAll scripts executed successfully.\033[0m"
else
    echo -e "\033[0;31mERROR: $error_counter script(s) failed.\033[0m"
    echo -e "\033[0;31mFailed scripts: ${error_scripts[*]}\033[0m"
    exit 1
fi