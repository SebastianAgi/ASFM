build:
	@cp /etc/nv_tegra_release .nv_tegra_release
	docker build -t seb_first_image .
	@rm -Rf .nv_tegra_release

.pull:
	mkdir ~/ros_ws/src/
	git -C ~/ros_ws/src clone -b 

run:
	@docker container stop seb_jetson_ZED || true && docker container rm seb_jetson_ZED || true
	docker run \
		-it \ 						#enables the container to be interactive for use 
		-e "DISPLAY" \ 					#Lets container access to the X server (?)
		-e "QT_X11_NO_MITSHM=1" \ 			#enabbles QT software (?)
		-e "XAUTHORITY=${XAUTH}" \ 			#authenticating container access to X server
		-e ROS_MASTER_URI \				#set ROS_MASTER_URI to what it is on local pc
		-e ROS_IP \					#ROS_IP = local ROS_IP
		-v ~/.Xauthority:/root/.Xauthority:rw \ 	#mounts ~/.Xauthority to root in the container with rw permission
		-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \ 	#mounts /tmp/.X11-unix in the container with rw permission
		--network host \ 				#set containers network stack the same as the host's (use host's network)
		--privileged \ 					#lets container user pc hardware
		--runtime=nvidia \ 				#set container runtime to nvidia's GPU-accelerated container runtime
		--name seb_jetson_ZED \ 			#container name 
		seb_first_image 				#name again (?)
