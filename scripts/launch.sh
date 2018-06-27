#!/bin/bash

# # Red error text color (ANSI escape code)
# ERROR='\033[0;31m'

# # Get vrserver process identifier (PID)
# PID=$(pgrep -f vrserver)

# Steam runtime and catkin workspace directory paths
STEAM_RUNTIME=$HOME/.steam/steam/ubuntu12_32/steam-runtime
CATKIN_WS=$HOME/catkin_ws

# # Check if PID is empty
# if [ -n "$PID" ];
#     then
#         # Check if process is running
#         if (ps -p $PID > /dev/null)
#             then
                # Run node through Steam
                $STEAM_RUNTIME/run.sh $CATKIN_WS/devel/lib/vr_ros/vr_ros_node
#             else
#                 echo "$ERROR Unable to launch the vr_ros_node: vrserver is not running" 1>&2
#         fi
#     else
#         echo "$ERROR Unable to launch the vr_ros_node: vrserver is not running" 1>&2
# fi