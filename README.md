# touch_detect_wsg50110_integration
This repo has documents related to the integration of TouchDetect and WSG 50-110 robotic gripper

## Preparation
Copy / Move both files (demo_td_grip_control_v2.lua + td_control_fncs_v2.lua) to usr available directory on wsg50-100 gripper.
Both files must be in same path since demo... requires td_control... .

Eventually: depending on system which calls te commands over tcp/ip, disable checksum checks on wsg50-100 in the settings. 

## Run
In interactive scripting tab on wsg webinterface open "demo_td_grip_control_v2.lua" and run.
Configuration FLAGS (INIT / DEFAULTS ) in the "demo_td_grip_control_v2" allow for adaptions like:
* Debug and Info output in webinterface console
* Finger Index Orientation
* Read Data Timeout tries
* Grip control parameters
* Enabling conditional release

See Code and "-- comments" for details.

## td_control_fncs_v2
Outsourced helper and necessary function blocks including:
* sensor data variables and getter
* aqcuire complete data frame from both fingers
* | +> communication and getting sensor frame over finger-port 
* | +> combining finger read bytes to data array representation
* | +> flip array in x/y
* | +> rotate array in direction
* check data array on defined "ok" conditions
* | +> check spec. array on number of active nodes
* print data arrays to webinterface console

" | +> (independent function but also called by higher level function above) "
