#!/bin/sh

# Adds the contents of license.txt to the top of all (.h | .cc) files that do
# not currently have a license. Run from the base directory.
FILES=$(find . -iname "*.h" -o -iname "*.cc" | grep -v libraries)

for file in $FILES
do
    if ! grep -q Copyright $file
    then
	echo "Modifying file: $file"
	cat license.txt $file > $file.new && mv $file.new $file
    fi
done
