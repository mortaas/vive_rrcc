# Contains information about the currently tracked devices.
# 
# The header contains the fixed frame, which each
# tracked device is associated with.

# Define classes of tracked devices:
# The ID was not valid.
uint8 INVALID=0
# Head-Mounted Displays
uint8 HMD=1
# Tracked controllers
uint8 CONTROLLER=2
# Generic trackers, similar to controllers
uint8 TRACKER=3
# Camera and base stations that serve as tracking reference points
uint8 LIGHTHOUSE=4


Header header
# Number of tracked devices
uint8 device_count
# Classes of tracked devices
uint8[] device_classes
# Frames associated with each tracked device
string[] device_frames