#!/bin/bash

echo "Do you really want to replace the files in the ROS phidgets_drivers package? (Y/N)"
read -p "" answer

if [ "${answer}" == "Y" ] || [ "${answer}" == "y" ]; then
	rm -r ~/catkin_ws/src/phidgets_drivers/phidgets_api
	rm -r ~/catkin_ws/src/phidgets_drivers/phidgets_motors

	cp -r phidgets_api phidgets_motors ~/catkin_ws/src/phidgets_drivers

	echo "Replacing done"
else
	echo "Replacing aborted"
fi

exit 0

