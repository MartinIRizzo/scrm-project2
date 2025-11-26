#!/bin/bash

if [ $# -ne 2 ]; then
    echo "Usage: $0 <process-name> <output-file>"
    exit 1;
fi

PROCESS="$1"
FILE="$2"

echo "Monitoring process: '$PROCESS'"
echo "Press CTRL+C to stop"

while true; do
    CPU=$(top -b -n1 | grep "$PROCESS" | awk '{print $9}')

    echo "$CPU" >> "$FILE"

    sleep 2
done