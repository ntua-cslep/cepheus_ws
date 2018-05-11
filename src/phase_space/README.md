# Phase-Space ROS package

Software related to the use of the Phase Space motion tracking.

## How to start the system

1. Turn on the phase-space system and login as:
`username: demo`
`password:demo`
`startx`

2. Calibrate the phase space system, open a terminal and type
`cd phasespace`
`./calib`

3. Follow the instructions on the window, at somepoint it will ask you to connect the calibration wand, so look for it.

There is a [guide](doc/guida_PS.pdf) written in italian if you want to know more details.

## Marker info and visualization only

Run the command below in your computer (you can set inside the launch file to turn on visualization or not, visualization can be run in a different computer now): 

`roslaunch phase_space phase_space.launch`

Rviz should open with a custom configuration displaying the marker and ids, useful for debug and setup scenarios.


## Tracking known objects with known-positioned leds


You need to:
- Write an OBJECT.YAML configuration file in the `config` folder with the leds id and local coordinates of them, and note the `OBJECT_leds:` at the top, it is important. A CAD tool is advised if the object model is available, and double check! the correspondence id-coordinates using the master tool in the phasepace.
- Provide an OBJECT.STL model (only for online visualization), and place it in the `urdf/mesh` folder
- Edit `track_object.launch`, specifically the line `<arg name="object" default="OBJECT" />`

IMPORTANT: note that OBJECT is repeated in the three cases.

Finally, run:

`roslaunch phase_space track_object.launch object:=OBJECT`

An Rviz window should open with almost everything configured, you just need to update the robot model display with the parameter: `/OBJECT/robot_description` and you should see the mesh flying around.

To track multiple objects, include `track_object_launch` within a group with a namespace and fill the arguments properly for each object you want to track.


## Tracking unknown objects using the star

This requires a calibration procedure in order to known the transformation between the star and the object. The procedure requires an object pose estimation to broadcast an object pose w.r.t. a camera, camera phase space calibration to know where the camera is w.r.t. phase space system, track a known object with the previous procedure, for instance the [star](urdf/mesh/star.stl), and compute the resulting tracker/object transform that closes the loop. You can use our [calibration](https://github.com/CentroEPiaggio/calibration) package to do that.

