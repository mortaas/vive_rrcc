# Contains information about the currently tracked devices.
# 
# The header contains the fixed frame, which each
# tracked device is associated with.


# Invalid value for class/role
uint8 INVALID=0

# Define classes of tracked devices:
# Head-Mounted Displays
uint8 HMD=1
# Tracked controllers
uint8 CONTROLLER=2
# Generic trackers, similar to controllers
uint8 TRACKER=3
# Camera and base stations that serve as tracking reference points
uint8 LIGHTHOUSE=4

# Define roles of tracked devices:
# Tracked device associated with the left hand
uint8 LEFT_HAND=1
# Tracked device associated with the right hand
uint8 RIGHT_HAND=2
# Tracked device is opting out of left/right hand selection
uint8 OPT_OUT=3
# Tracked device is a treadmill
uint8 TREADMILL=4


Header header
# Number of tracked devices
uint8 device_count
# Classes of tracked devices
uint8[] device_classes
# Roles of tracked devices
uint8[] device_roles
# Frames associated with each tracked device
string[] device_frames