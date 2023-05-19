#!/bin/bash

nr_frames=${1:-1}

v4l2-ctl --set-ctrl=operating_mode=10 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=phase_depth_bits=4 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=ab_bits=6 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=confidence_bits=0 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=ab_averaging=0 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=depth_enable=0 -d /dev/v4l-subdev1
v4l2-ctl --device /dev/video0 --set-fmt-video=width=2048,height=3328,pixelformat=BA81 --stream-mmap --stream-to=mode10.bin --stream-count=$nr_frames
