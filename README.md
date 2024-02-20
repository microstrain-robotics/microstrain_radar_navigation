# microstrain_radar_navigation
# Common
TTYACM0 - CV7_INS
TTYACM1 - Ublox ZED-F9P

# Docker
docker build -t <name> .
xhost local:root
docker run --rm -it --env DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged -v /dev:/dev --net=host <name>
