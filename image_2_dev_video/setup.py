#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['image_2_dev_video', 'image_2_dev_video_ros'],
 package_dir={'image_2_dev_video': 'common/src/image_2_dev_video', 'image_2_dev_video_ros': 'ros/src/image_2_dev_video_ros'}
)

setup(**d)
