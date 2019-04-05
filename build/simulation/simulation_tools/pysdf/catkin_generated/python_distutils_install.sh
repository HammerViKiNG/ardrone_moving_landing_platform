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

echo_and_run cd "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/simulation/simulation_tools/pysdf"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hammerviking/robotic_software/ardrone_moving_landing_platform/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hammerviking/robotic_software/ardrone_moving_landing_platform/install/lib/python2.7/dist-packages:/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build" \
    "/usr/bin/python" \
    "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/simulation/simulation_tools/pysdf/setup.py" \
    build --build-base "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/simulation/simulation_tools/pysdf" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/hammerviking/robotic_software/ardrone_moving_landing_platform/install" --install-scripts="/home/hammerviking/robotic_software/ardrone_moving_landing_platform/install/bin"
