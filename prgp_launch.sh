#!/bin/bash

function cpp_func {
	
    /home/prgpdt/catkin_ws/devel/lib/prgp_piswarmcom/prgp_piswarmcom

}
gnome-terminal -e '/bin/bash -c "roscore"' &
sleep 1
gnome-terminal -e '/bin/bash -c "rosrun prgp_piswarmcom prgp_piswarmcom"' &
while read line; do
    if [ "$line" == "STARTDRONE" ]; then
	
	gnome-terminal -e '/bin/bash -c "rosrun ardrone_autonomy ardrone_driver"' &
	sleep 10
	gnome-terminal -e '/bin/bash -c "rosrun tum_ardrone drone_stateestimation"' &
	gnome-terminal -e '/bin/bash -c "rosrun tum_ardrone drone_autopilot"' &
	gnome-terminal -e '/bin/bash -c "rosrun tum_ardrone drone_gui"' &
	gnome-terminal -e '/bin/bash -c "rosrun prgp_ardrone prgp_ardrone"' &
        break    
    fi

done < <(cpp_func)
