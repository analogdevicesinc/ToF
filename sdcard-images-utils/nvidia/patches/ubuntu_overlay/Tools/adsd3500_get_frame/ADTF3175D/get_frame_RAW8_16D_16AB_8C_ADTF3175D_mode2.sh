#!/bin/bash

nr_frames=${1:-1}

v4l2-ctl --set-ctrl=operating_mode=2 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=phase_depth_bits=6 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=ab_bits=6 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=confidence_bits=2 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=ab_averaging=1 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=depth_enable=1 -d /dev/v4l-subdev1
v4l2-ctl --device /dev/video0 --set-fmt-video=width=2560,height=512,pixelformat=RGGB --stream-mmap --stream-to=mode2.bin --stream-count=$nr_frames
