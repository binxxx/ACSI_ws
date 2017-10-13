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

echo_and_run cd "/home/binx/Documents/Research/Crazy/src/hrl-kdl/hrl_geom"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/binx/Documents/Research/Crazy/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/binx/Documents/Research/Crazy/install/lib/python2.7/dist-packages:/home/binx/Documents/Research/Crazy/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/binx/Documents/Research/Crazy/build" \
    "/usr/bin/python" \
    "/home/binx/Documents/Research/Crazy/src/hrl-kdl/hrl_geom/setup.py" \
    build --build-base "/home/binx/Documents/Research/Crazy/build/hrl-kdl/hrl_geom" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/binx/Documents/Research/Crazy/install" --install-scripts="/home/binx/Documents/Research/Crazy/install/bin"
