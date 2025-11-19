HOW TO RUN:

Navigate into the provided miro-docker repository and start the miro-docker container

`cd miro-docker`

`./miro-docker.sh start`

wait for the miro docker container and image to be made just ress enter if asked for names

launch the miro hub terminal

`miro-hub term`

navigate to mdk/catkin_ws/src within the miro-docker container
run pwd to find the full path

navigate into "feed_the_miro_pkg" our repository in another terminal
run `pwd` to find the full path

copy the feed the miro package into the docker container using

`docker cp <output of second pwd> miro-docker:<output of first pwd>`

check that the ros1 package feed_the_miro_pkg has been copied into miro-docker

make the yolo container and run the yolo node

set miro mode to robot
configure the ip for the miro and computer

connect to dia lab

connect to the miro

check the miro gui works

run the camera_reader.py within the copy of the ros1 package in miro-docker