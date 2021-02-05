#!/bin/bash
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
filename=$(ls $SCRIPTPATH/src/cepheus_robot/bags -t | head -1)
printf "Exporting from topics from latest bag file: %s\n" "$filename"
printf "[ 1/10] exporting secs.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /secs > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.secs.txt"
printf "exported\n"
printf "[ 2/10] exporting set_ls_effort.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /set_left_shoulder_effort > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.set_ls_effort.txt"
printf "exported\n"
printf "[ 3/10] exporting set_le_effort.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /set_left_elbow_effort > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.set_le_effort.txt"
printf "exported\n"
printf "[ 4/10] exporting set_re_effort.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /set_right_elbow_effort > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.set_re_effort.txt"
printf "exported\n"
printf "[ 5/10] exporting read_ls_position.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /read_left_shoulder_position > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.read_ls_position.txt"
printf "exported\n"
printf "[ 6/10] exporting read_le_position.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /read_left_elbow_position > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.read_le_position.txt"
printf "exported\n"
printf "[ 7/10] exporting read_re_position.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /read_right_elbow_position > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.read_re_position.txt"
printf "exported\n"
printf "[ 8/10] exporting read_ls_velocity.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /read_left_shoulder_velocity > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.read_ls_velocity.txt"
printf "exported\n"
printf "[ 9/10] exporting read_le_velocity.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /read_left_elbow_velocity > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.read_le_velocity.txt"
printf "exported\n"
printf "[10/10] exporting read_re_velocity.txt... "
rostopic echo -b "$SCRIPTPATH/src/cepheus_robot/bags/$filename" -p /read_right_elbow_velocity > "$SCRIPTPATH/src/cepheus_robot/bags/$filename.read_re_velocity.txt"
printf "exported\n"
python3 export_data.py
