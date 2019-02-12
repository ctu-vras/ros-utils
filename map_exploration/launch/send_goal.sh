#!/bin/bash
rostopic pub -r 2 mapping/goals nav_msgs/Path  '{header: {frame_id: camera_depth_optical_frame}, poses: [ {pose: {position: {x: 0, y: 0}}}]  }'


