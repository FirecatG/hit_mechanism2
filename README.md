# Usage:

  - **simulation(ROS2 environment required)**

```
	mkdir -p ~/cam_ws/src
	cd ~/cam_ws/src
	git clone git@github.com:FirecatG/hit_mechanism2.git

	cd ~/cam_ws/
	rosdep install --from-paths . --ignore-src -y
	colcon build
	source install/setup.bash

	ros2 launch hit_mechanism2 main.launch.py
```




- **plot (with python)**
  `python3 scripts/main.py`