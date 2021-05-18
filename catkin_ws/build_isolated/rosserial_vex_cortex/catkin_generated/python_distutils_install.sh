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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/mateusz/ros/catkin_ws/src/rosserial/rosserial_vex_cortex"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mateusz/ros/catkin_ws/install_isolated/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mateusz/ros/catkin_ws/install_isolated/lib/python2.7/dist-packages:/home/mateusz/ros/catkin_ws/build_isolated/rosserial_vex_cortex/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mateusz/ros/catkin_ws/build_isolated/rosserial_vex_cortex" \
    "/usr/bin/python2" \
    "/home/mateusz/ros/catkin_ws/src/rosserial/rosserial_vex_cortex/setup.py" \
     \
    build --build-base "/home/mateusz/ros/catkin_ws/build_isolated/rosserial_vex_cortex" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/mateusz/ros/catkin_ws/install_isolated" --install-scripts="/home/mateusz/ros/catkin_ws/install_isolated/bin"
