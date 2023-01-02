# cras_bag_tools

Czech-army knife for working with ROS BAG files.

## Python Modules

Several useful functions are exported as Python submodules as module `cras_bag_tools`:

- [`fix_msg_defs`](https://docs.ros.org/en/api/cras_bag_tools/html/cras_bag_tools.html#module-cras_bag_tools.fix_msg_defs): Fix message definitions in a bag file according to local definitions.
- [`topic_set`](https://docs.ros.org/en/api/cras_bag_tools/html/cras_bag_tools.html#module-cras_bag_tools.topic_set): Efficient data structure to hold a static set of topics with super-fast is-in-set queries.
- [`tqdm_bag`](https://docs.ros.org/en/api/cras_bag_tools/html/cras_bag_tools.html#module-cras_bag_tools.tqdm_bag): Bag file reader that shows progressbars when loading index or reading messages.

## Scripts

### merge_bag

Merge two or more bag files into one.

#### Usage

    rosrun cras_bag_tools merge_bag [-v] [-c] [-b] OUT_BAG IN_BAG [IN_BAG ...]

- `OUT_BAG`: Output bag file.
- `IN_BAG`: Input bag file(s).
- `-v`, `--verbose`: Verbose output.
- `-c`, `--compress`: Compress output bag with LZ4 compression.
- `-b`, `--bz2`: Compress output bag with BZ2 compression.

### size_per_topic

Print total cumulative serialized message size per topic.

#### Usage

    rosrun cras_bag_tools size_per_topic [-c] [-a] BAG

- `BAG`: Bag file.
- `-c`, `--csv`: Output as CSV.
- `-a`, `--sort-alphabetical`: Sort by topic names (default is by topic sizes).

### fix_msg_defs

Sometimes it can happen that wrong textual definitions of messages are stored in bag files (although the MD5 sums are correct).
Such bag files can have problems when they are processed using scripts like `rosbag` etc.
This script fixes the definitions using the definitions from the local catkin workspace.

**The fix is done inplace, so it changes the original bag file.**

#### Usage

    rosrun cras_bag_tools fix_msg_defs BAGFILE [TOPIC [TOPIC ...]]

- `BAGFILE`: The bag file to fix.
- `TOPIC`: Any number of topic names to fix. If not set, all topics are fixed. The topics are sensitive to the starting slash (not) being present.

> **Note**
> If there is a mismatch between the stored and local MD5 sum of the message definition, it will not be fixed and a warning will be printed.

> **Note**
> If a definition cannot be found locally for some message type, a warning will be printed and this definition will not be fixed.
