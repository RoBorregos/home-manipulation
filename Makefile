# ----------------------------------------------------------------------
#  Robocup@Home ROS Noetic Docker Development
# ----------------------------------------------------------------------

#: Builds a Docker image with the corresponding Dockerfile file

# ----------------------------BUILD------------------------------------
# ---------Manipulation----------
# No GPU
manipulation.build:
	@./docker/scripts/build.bash --area=manipulation

# CUDA 11.8 x86_64
manipulation.build.cuda:
	@./docker/scripts/build.bash --area=manipulation --use-cuda

# Jetson devices
manipulation.build.jetson:
	@./docker/scripts/build.bash --area=manipulation --jetson-l4t=35.4.1

# Manipulation GPU + ZED + OakD
manipulation.build.full:
	@./docker/scripts/build.bash --area=manipulation-full --use-cuda

# ----------------------------CREATE------------------------------------

manipulation.create:
	@./docker/scripts/run.bash --area=manipulation --volumes=$(volumes) --name=$(name)

manipulation.create.cuda:
	@./docker/scripts/run.bash --area=manipulation --use-cuda --volumes=$(volumes) --name=$(name)

manipulation.create.full:
	@./docker/scripts/run.bash --area=manipulation-full --use-cuda --volumes=$(volumes) --name=$(name)

# For jetpack version 35.4.1, jetson images are special in the sense that they are specific to the jetpack version
manipulation.create.jetson:
	@./docker/scripts/run.bash --area=manipulation --jetson-l4t=35.4.1 --volumes=$(volumes) --name=$(name)

# ----------------------------START------------------------------------
# Start containers
manipulation.up:
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@docker start home-manipulation

manipulation.up.full:
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@docker start home-manipulation-full

# ----------------------------STOP------------------------------------
# Stop containers
manipulation.down:
	@docker stop home-manipulation 

manipulation.down.full:
	@docker stop home-manipulation-full

# ----------------------------RESTART------------------------------------
# Restart containers
manipulation.restart:
	@docker restart home-manipulation 

# ----------------------------LOGS------------------------------------
# Logs of the container
manipulation.logs:
	@docker logs --tail 50 home-manipulation

# ----------------------------SHELL------------------------------------
# Fires up a bash session inside the container
manipulation.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) home-manipulation bash

manipulation.shell.full:
	@docker exec -it --user $(shell id -u):$(shell id -g) home-manipulation-full bash

# ----------------------------REMOVE------------------------------------
# Remove container
manipulation.remove:
	@docker container rm home-manipulation

manipulation.remove.full:
	@docker container rm home-manipulation-full

# ----------------------------------------------------------------------
#  General Docker Utilities

#: Show a list of images.
list-images:
	@docker image ls

#: Show a list of containers.
list-containers:
	@docker container ls -as
