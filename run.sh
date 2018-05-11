#!/bin/bash

handle() {
	kill -2 "$child" 
}

trap "handle" SIGINT

echo "" > stdout.log
echo "" > stderr.log

echo "Running trap handler..."
 ./trap.sh &
child=$!

echo "Running roslaunch..."

$1 $2 $3 > \
	>(tee -a stdout.log) 2> \
	>(tee -a stderr.log >/dev/null)

echo "Waiting for child..."
wait "$child"
