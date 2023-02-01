all:
	colcon build --symlink-install
	. install/setup.bash

clean:
	rm -rf build install log

pub:
	ros2 run driver controller_pub

sub:
	ros2 run driver drive_receiver_sub