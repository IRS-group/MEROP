#!/bin/bash

ffplay -f v4l2 /dev/video11 && ffplay -f v4l2 /dev/video12 && ffplay -f v4l2 /dev/video13
reset # clean terminal
rosnode machine merop-vm | grep image_2_dev_video

