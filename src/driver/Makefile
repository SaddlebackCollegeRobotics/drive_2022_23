all:
	colcon build --symlink-install

clean:
	rm -rf build install log

pub:
	@echo ". install/setup.bash"
	ros2 run driver controller_pub

sub:
	@echo ". install/setup.bash"
	ros2 run driver drive_receiver_sub