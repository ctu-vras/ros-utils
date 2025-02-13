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

### extract_images

Extract image topics from bag file to image or video files.

#### Usage

    rosrun cras_bag_tools extract_images [-h] [--out-format OUT_FORMAT] [--verbose] bag_file output_dir [image_topics [image_topics ...]]

* `bag_file`: The bag to read.
* `output_dir`: Directory where all generated files should be stored.
* `image_topics`: Zero or more topics to convert. If zero, all image topics are converted. Each image topic can be
  followed by additional configuration in the form
  `TOPIC:OUT_FORMAT:COMPRESSION:ENCODING:IN_FPS:OUT_FPS:PIX_FMT` (only `TOPIC` is required)
    * `OUT_FORMAT`: Format of the exported images/video (e.g. `jpg`, `mp4` etc.). Defaults to `--out-format`.
    * `COMPRESSION`: Compression level. The meaning depends on `OUT_FORMAT`. JPG has 0-100, PNG 0-9, MP4 0-51 (CRF).
    * `ENCODING`: Pixel format to convert the images to. E.g. `bgr8` or `mono8`. `passthrough` means keeping the format.
      Empty string retains the format for raw images and converts compressed images to the recorded raw format
      in their `.format` field if it is available. Defaults to `passthrough`.
    * Following options are for video formats only.
    * `IN_FPS`: Framerate of the topic images. If not set, it is estimated from the frequency of the messages in the bag.
      You can set the FPS higher than the original, effectively speeding up the video.
    * `OUT_FPS`: Output framerate. Defaults to `IN_FPS`.
    * `PIX_FMT`: The `-pix_fmt` output option for ffmpeg, i.e. the pixel format of the video stream. Defaults to
      `yuvj420p`.
* `-h`: Shows help.
* `--out-format`: Default format for all topics (e.g. `jpg`, `mp4` etc.). Defaults to `jpg`.
* `--verbose`: Print various details during execution.

#### Example command

```bash
# Convert all image topics in the bag to a series of JPEG images in folder imgs/
rosrun cras_bag_tools extract_images spot_2022-10-27-10-35-46.video.bag imgs

# Convert topic /rviz/fixed_image/compressed to an MP4 video
rosrun cras_bag_tools extract_images spot_2022-10-27-10-35-46.video.bag . /rviz/fixed_image/compressed:mp4:23:passthrough:25
```

### filter_bag

Process bag files with a configured set of filters.

#### Usage

    usage: filter_bag [-h] [-c CONFIG [CONFIG ...]] [-o OUT_FORMAT] [--lz4] [--bz2] [--no-copy-params] [--list-yaml-keys] [--list-filters] [--merge-initial-static-tf [DURATION]]
                      [--throttle TOPIC RATE [TOPIC RATE ...]] [-i INCLUDE_TOPICS [INCLUDE_TOPICS ...]] [-e EXCLUDE_TOPICS [EXCLUDE_TOPICS ...]] [--include-types INCLUDE_TYPES [INCLUDE_TYPES ...]]
                      [--exclude-types EXCLUDE_TYPES [EXCLUDE_TYPES ...]] [--include-tf-parents INCLUDE_TF_PARENTS [INCLUDE_TF_PARENTS ...]]
                      [--exclude-tf-parents EXCLUDE_TF_PARENTS [EXCLUDE_TF_PARENTS ...]] [--include-tf-children INCLUDE_TF_CHILDREN [INCLUDE_TF_CHILDREN ...]]
                      [--exclude-tf-children EXCLUDE_TF_CHILDREN [EXCLUDE_TF_CHILDREN ...]] [--decompress-images] [--max-message-size MAX_MESSAGE_SIZE]
                      [bags [bags ...]]


Positional arguments:

* `bags`: The list of bags to process.

Optional arguments:
* `--list-yaml-keys`: Print a list of all available YAML top-level keys provided by filters.
* `--list-filters`: Print a list of all available filters.
* `--no-copy-params`: If set, no .params file will be copied
* `-i INCLUDE_TOPICS [INCLUDE_TOPICS ...]`, `--include-topics INCLUDE_TOPICS [INCLUDE_TOPICS ...]`: Retain only
  these topics
* `-e EXCLUDE_TOPICS [EXCLUDE_TOPICS ...]`, `--exclude-topics EXCLUDE_TOPICS [EXCLUDE_TOPICS ...]`: Remove these topics
* `--include-types INCLUDE_TYPES [INCLUDE_TYPES ...]`: Retain only messages of these types
* `--exclude-types EXCLUDE_TYPES [EXCLUDE_TYPES ...]`: Remove messages of these types
* `--throttle TOPIC RATE [TOPIC RATE ...], --hz TOPIC RATE [TOPIC RATE ...]`: Throttle messages. This argument should be
  an even-sized list of pairs `[TOPIC RATE]`. 
* `--include-tf-parents INCLUDE_TF_PARENTS [INCLUDE_TF_PARENTS ...]`: Retain only TFs with these frames as parents
* `--exclude-tf-parents EXCLUDE_TF_PARENTS [EXCLUDE_TF_PARENTS ...]`: Remove TFs with these frames as parents
* `--include-tf-children INCLUDE_TF_CHILDREN [INCLUDE_TF_CHILDREN ...]`: Retain only TFs with these frames as children
* `--exclude-tf-children EXCLUDE_TF_CHILDREN [EXCLUDE_TF_CHILDREN ...]`: Remove TFs with these frames as children
* `--max-message-size MAX_MESSAGE_SIZE`: Remove all messages larger than this size `[B]`
* `--merge-initial-static-tf [DURATION]`: Merge a few initial static TFs into one. DURATION specifies the duration of
  the initial bag section to be considered for the merging. DURATION defaults to 5 secs.
* `--decompress-images`: Decompress all images
* `-c CONFIG [CONFIG ...]`, `--config CONFIG [CONFIG ...]`: YAML configs of filters
* `-o OUT_FORMAT`, `--out-format OUT_FORMAT`: Template for naming the output bag. Defaults to `{name}.proc{ext}`. 
  Relative paths will put the bag relative to current directory (not relative to the location of the source bag).
  The format string can utilize variables `dirname` (directory of the source bag), `name` (name of the source bag
  without extension), `ext` (extension of the source bag (should be `.bag`)).
* `--lz4`: Compress the bag using LZ4 compression.
* `--bz2`: Compress the bag using BZ2 compression (warning: this compression is very slow).

The YAML config files can contain the same keys as the optional CLI arguments (list all of them via `--list-yaml-keys`.
Additionally, YAML configs can contain key `filters` which is a list of additional filters. Each filter is defined as a
dict with the filter class as key and filter configuration as value. See the `config/` folder for examples. List all
available filters via `--list-filters`.

Other filters can be defined by 3rd-party packages via pluginlib. The package has to
`<exec_depend>cras_bag_tools</exec_depend>` and it has to put this line in its `<export>` tag in package.xml:
`<cras_bag_tools filters="$PACKAGE.$MODULE" />`. With this in place, `filter_bag` will search the specified module
for all classes that subclass `cras_bag_tools.MessageFilter` and it will provide these as additional filters.

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
