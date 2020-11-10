#!/usr/bin/env python

import rospkg

rospack = rospkg.RosPack()
#rospack.list_pkgs()

path_nav = rospack.get_path('cube_navigation')
path_ctr = rospack.get_path('cube_control')
path_bri = rospack.get_path('cube_bringup')

# check log directory is exist
from os.path import expanduser
log_dir_name = expanduser("~") + "/.ros/navlog"
import os
if not os.path.exists(log_dir_name):
  os.makedirs(log_dir_name)

# save log
import shutil
from datetime import datetime
dir_name = datetime.now().strftime('%Y%m%d_%H%M%S')
shutil.copytree(path_nav + "/launch", log_dir_name + "/" + dir_name + "/cube_navigation/launch")
shutil.copytree(path_nav + "/param", log_dir_name + "/" + dir_name + "/cube_navigation/param")
shutil.copytree(path_ctr + "/launch", log_dir_name + "/" + dir_name + "/cube_control/launch")
shutil.copytree(path_ctr + "/config", log_dir_name + "/" + dir_name + "/cube_control/config")
shutil.copytree(path_bri + "/launch", log_dir_name + "/" + dir_name + "/cube_bringup/launch")
shutil.copytree(path_bri + "/config", log_dir_name + "/" + dir_name + "/cube_bringup/config")

