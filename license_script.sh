#!/bin/sh

# Replaces the license in files with the current version of license.txt. This is
# useful when updating the license.

# First, find the last line of the license.
FILES=$(ack -l "Author: Chris Sweeney" | ack -v build | ack -v doc | ack '.*\.(cc|h)+$')
for file in $FILES
do
    LINE_NO=`grep -m=1 -ne "Author: Chris Sweeney" $file | sed -n 's/^\([0-9]*\)[:].*/\1/p'`
    LINE_NO=$(($LINE_NO + 2))
    echo "Modifying file: $file at $LINE_NO"
    tail +$LINE_NO $file > $file.new && mv $file.new $file
    cat license.txt $file > $file.new && mv $file.new $file
done
