# cras_bag_tools

Various utilities to work with bag files

## bag_fix_msg_defs

Sometimes it can happen that wrong textual definitions of messages are stored in bag files (although the MD5 sums are correct).
Such bag files can have problems when they are processed using scripts like `rosbag` etc.
This script fixes the definitions using the definitions from the local catkin workspace.

**The fix is done inplace, so it changes the original bag file.**

### Usage

    rosrun cras_bag_tools bag_fix_msg_defs BAGFILE [TOPIC [TOPIC ...]]

- `BAGFILE`: The bag file to fix.
- `TOPIC`: Any number of topic names to fix. If not set, all topics are fixed. The topics are sensitive to the starting slash (not) being present.

### Note

If there is a mismatch between the stored and local MD5 sum of the message definition, it will not be fixed and a warning will be printed.

If a definition cannot be found locally for some message type, a warning will be printed and this definition will not be fixed.