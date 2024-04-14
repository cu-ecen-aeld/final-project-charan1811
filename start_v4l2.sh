#!/bin/bash

# Check if the number of arguments is correct
if [ "$#" -gt 0 ]; then
    echo "./start_v4l2.sh"
    exit 1
fi

make clean

# Assign arguments to variables
writefile1="$(pwd)"
echo $writefile1
writefile="$writefile1/demo"

cd "$writefile1"

echo "changed directory"

# Check if writefile is specified if not print an error message stating invalid path by returing 1
if [ -d "$writefile" ]; then
    echo "directory present\n"
    rm -rf writefile
fi

echo "creating directory"
echo $writefile

#mkdir -p "demo"
# Create the directory path if it doesn't exist for copying the content from the string
mkdir -p "$writefile"

# Check if the file was created successfully and prints if it can't create a file
if [ "$?" -ne 0 ]; then
    echo "Error: Could not create the file '$writefile'."
    exit 1
fi

make

sudo ./capt

# Print success message and Display the results to the user
echo "File created successfully: $writefile"

# Exit the script with a success status
exit 0
