build:
	@cp /etc/nv_tegra_release .nv_tegra_release
	docker build -t seb_first_image .
	@rm -Rf .nv_tegra_release
	@docker run \
		--rm \
		-v ~/Documents/social_navigation:/root/ros_ws/src/social_navigation:rw \
		--name seb_jetson_ZED \
		seb_first_image \
		bash -c "source /opt/ros/noetic/setup.bash; roscore"
	@sleep 5
	@docker exec -it seb_jetson_ZED bash -c "source /opt/ros/noetic/setup.bash; cd /root/ros_ws; catkin_make"
	@$(MAKE) -s -f $(THIS_FILE) stop

run:
	@docker container stop seb_jetson_ZED || true && docker container rm seb_jetson_ZED || true
	docker run \
		-it \																		#enables the container to be interactive for use 
		-e "DISPLAY" \																#Lets container access to the X server (?)
		-e "QT_X11_NO_MITSHM=1" \													#enabbles QT software (?)
		-e "XAUTHORITY=${XAUTH}" \													#authenticating container access to X server
		-e ROS_MASTER_URI \															#set ROS_MASTER_URI to what it is on local pc
		-e ROS_IP \																	#ROS_IP = local ROS_IP
		-v ~/.Xauthority:/root/.Xauthority:rw \										#mounts ~/.Xauthority to root in the container with rw permission
		-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \										#mounts /tmp/.X11-unix in the container with rw permission
		-v ~/Documents/social_navigation:/root/ros_ws/src/social_navigation:rw
		--network host \															#set containers network stack the same as the host's (use host's network)
		--privileged \																#lets container user pc hardware
		--runtime=nvidia \															#set container runtime to nvidia's GPU-accelerated container runtime
		--name seb_jetson_ZED \ 													#container name 
		seb_first_image 															#name again (?)

stop:
	@docker stop seb_jetson_ZED >> /dev/null
