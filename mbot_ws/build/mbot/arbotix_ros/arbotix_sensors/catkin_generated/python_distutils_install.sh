#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/songsong/catkin_workspace/mbot_ws/src/mbot/arbotix_ros/arbotix_sensors"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/songsong/catkin_workspace/mbot_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/songsong/catkin_workspace/mbot_ws/install/lib/python3/dist-packages:/home/songsong/catkin_workspace/mbot_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/songsong/catkin_workspace/mbot_ws/build" \
    "/home/songsong/Documents/Softwares/anaconda3/bin/python" \
    "/home/songsong/catkin_workspace/mbot_ws/src/mbot/arbotix_ros/arbotix_sensors/setup.py" \
    build --build-base "/home/songsong/catkin_workspace/mbot_ws/build/mbot/arbotix_ros/arbotix_sensors" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/songsong/catkin_workspace/mbot_ws/install" --install-scripts="/home/songsong/catkin_workspace/mbot_ws/install/bin"
