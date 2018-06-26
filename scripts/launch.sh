#!/bin/bash

# Get PID of vrserver process
PID = $(pgrep -f vrserver)

# Steam runtime and catkin workspace directory path
STEAM = ~/.steam/steam/ubuntu12_32/steam-runtime
CATKIN_WS = ~/catkin_ws

if ($PID == "")
    then
        echo "Unable to launch vr_ros: vrserver is not running" 1>&2
    else
        if (ps -p $PID > /dev/null)
            then
                $STEAM/run.sh $CATKIN_WS/devel/lib/vr_ros/vr_ros_node
            else
                echo "Unable to launch vr_ros: vrserver is not running" 1>&2
        fi
fi