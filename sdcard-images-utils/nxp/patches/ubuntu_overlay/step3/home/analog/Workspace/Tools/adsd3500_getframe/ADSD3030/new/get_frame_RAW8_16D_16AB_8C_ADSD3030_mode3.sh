#!/bin/bash

nr_frames=${1:-1}

v4l2-ctl --set-ctrl=operating_mode=3 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=phase_depth_bits=6 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=ab_bits=6 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=confidence_bits=2 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=ab_averaging=1 -d /dev/v4l-subdev1
v4l2-ctl --set-ctrl=depth_enable=1 -d /dev/v4l-subdev1
v4l2-ctl --device /dev/video0 --set-fmt-video=width=1280,height=320,pixelformat=BA81 --stream-mmap --stream-to=mode3.bin --stream-count=$nr_frames
