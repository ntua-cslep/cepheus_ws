#!/bin/bash

ID=""
let errors=0
handle() {
	# Kill spawner
	ID=`rosnode info cepheus_controller_spawner | grep "Pid" | cut -d" " -f2`
	kill -2 $ID

	sleep 5

	# Kill roslaunch
	ID=`ps -A | grep "roslaunch" | head -n 1| sed -e 's/^[ \t]*//' | cut -d" " -f1`
	kill -2 $ID
	exit
}

trap "handle" SIGINT

#tail -n0 -F stderr.log | grep --line-buffered -m 1 -E "Error|ERROR" stderr.log
tail -n0 -F stderr.log | while read LINE
do
#	[[ "${LINE}" =~ Error|died|ERROR ]] \
	[[ "${LINE}" =~ Error|died ]] \
        && (( errors=errors+1 )) \
	&& echo "$(tput setaf 1)$LINE$(tput sgr0)" \
        && [[ "$errors" -eq 5 ]] \
        && handle
done
