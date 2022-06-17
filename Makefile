THIS_FILE := $(lastword $(MAKEFILE_LIST))

build:
	@docker container stop seb_jetson_ZED || true && docker container rm seb_jetson_ZED || true
	@cp /etc/nv_tegra_release .nv_tegra_release
	docker build -t seb_first_image .
	@rm -Rf .nv_tegra_release
	@docker run \
		--rm \
		--detach \
		-v ~/Documents/social_navigation:/root/ros_ws/src/social_navigation:rw \
		--name seb_jetson_ZED \
		seb_first_image \
		bash -c "source /opt/ros/noetic/setup.bash; roscore"
	@sleep 5s
	@docker exec -it seb_jetson_ZED bash -c "source /opt/ros/noetic/setup.bash; cd /root/ros_ws; catkin build"
	@$(MAKE) -s -f $(THIS_FILE) stop

run:
	@docker container stop seb_jetson_ZED || true && docker container rm seb_jetson_ZED || true
	docker run \
		-it \
		-e "DISPLAY" \
		-e "QT_X11_NO_MITSHM=1" \
		-e "XAUTHORITY=${XAUTH}" \
		-e ROS_MASTER_URI=http://10.0.0.158:11311 \
		-e ROS_IP \
		-v ~/.Xauthority:/root/.Xauthority:rw \
		-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		-v ~/Documents/social_navigation:/root/ros_ws/src/social_navigation:rw \
		--network host \
		--privileged \
		--runtime=nvidia \
		--name seb_jetson_ZED \
		seb_first_image

stop:
	@docker stop seb_jetson_ZED >> /dev/null