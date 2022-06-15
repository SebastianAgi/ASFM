build:
	@cp /etc/nv_tegra_release .nv_tegra_release
	docker build -t seb_first_image .
	@rm -Rf .nv_tegra_release

run:
	@docker container stop seb_jetson_ZED || true && docker container rm seb_jetson_ZED || true
	docker run \
		-it \ 						#enables the container to be interactive for use 
		-e "DISPLAY" \ 					#Lets container access to the X server (?)
		-e "QT_X11_NO_MITSHM=1" \ 			#enabbles QT software (?)
		-e "XAUTHORITY=${XAUTH}" \ 			#authenticating container access to X server
		-e ROS_MASTER_URI \				#set ROS_MASTER_URI to what it is on local pc
		-e ROS_IP \					#ROS_IP = local ROS_IP
		-v ~/.Xauthority:/root/.Xauthority:rw \ 	#gives read write permission to root volume
		-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \ 	#read-write perm to X11 (?)
		--network host \ 				#set containers network stack the same as the host's (use host's network)
		--privileged \ 					#lets container user pc hardware
		--runtime=nvidia \ 				#set container runtime to nvidia's GPU-accelerated container runtime
		--name seb_jetson_ZED \ 			#container name 
		seb_first_image 				#name again (?)
