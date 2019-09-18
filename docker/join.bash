#!/usr/bin/env bash
# From https://bitbucket.org/osrf/subt/src/gazebo9/docker/join.bash
#
# Typical usage: ./join.bash subt
#

IMG=$(basename $1)

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}")
docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES=`tput lines` -it ${containerid} bash
xhost -
