THIS_FILE := $(lastword $(MAKEFILE_LIST))

.PHONY: .pull .build start body_driver keyboard model stop

default:
	@$(MAKE) -s .pull
	@$(MAKE) -s .build

.pull:
	mkdir -p ~/Documents/sebastian/prl_spot/src/

.build:
	docker build --tag=personalroboticsimperial/prl:prl-spot .
	@docker run --detach --rm -v ~/Documents/sebastian/prl_spot:/catkin_ws:rw --name prl-spot personalroboticsimperial/prl:prl-spot bash -c "source /opt/ros/noetic/setup.bash; roscore"
	@sleep 5
	@docker exec -it prl-spot bash -c "source /opt/ros/noetic/setup.bash; cd /catkin_ws; rosdep install --from-paths src --ignore-src --rosdistro noetic -y; catkin_make --cmake-args -DBUILD_TYPE=Release"
	@$(MAKE) -s -f $(THIS_FILE) stop

start:
	@xhost +si:localuser:root >> /dev/null
	@docker run --detach --rm --net host --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/Documents/sebastian/prl_spot:/catkin_ws:rw --name prl-spot personalroboticsimperial/prl:prl-spot bash -c "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; roscore"
	@sleep 5

body_driver: start
	-@docker exec -it prl-spot bash -c "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; roslaunch spot_driver spot_interface.launch motors_on:="Y""
	@$(MAKE) -s -f $(THIS_FILE) stop  # this shouldn't be reached from above, it should stop it but just in case, we'll cleanly exit

keyboard:
	@docker exec -it prl-spot bash -c "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rosrun spot_driver keyboard_teleop.py"

pose:
	@docker exec -it prl-spot bash -c "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rosrun spot_driver get_pose.py"

velo:
	@docker exec -it prl-spot bash -c "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rosrun spot_driver velo.py"


trajectory:
	@docker exec -it prl-spot bash -c "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rosrun spot_driver trajectory.py"

model:
	@docker exec -it prl-spot bash -c "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; roslaunch spot_description robot_model.launch"

stop:
	@docker stop prl-spot >> /dev/null