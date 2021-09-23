# thi_modded_phidgets_drivers
This are the modded phidgets drivers files. 

# Replace Phidgets Files
To replace the original phidgets_drivers sub-package with this modded one
```console
$ git clone https://github.com/roboTHIx/thi_modded_phidgets_drivers
$ cd thi_modded_phidgets_drivers
$ chmod +x replace_phidgets_files.sh
$ ./replace_phidgets_files.sh
```
After this, you will be asked if you really want to replace the original phidgets_drivers files. If you answer with "Y" or "y", the files in the ~/catkin_ws/src/phidgets_drivers folder will be replaced.
Bild your workspace again
```console
$ cd ~/catkin_ws
$ catkin_make
```
