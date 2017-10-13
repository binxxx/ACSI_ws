execute_process(COMMAND "/home/binx/Documents/Research/Crazy/build/hrl-kdl/hrl_geom/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/binx/Documents/Research/Crazy/build/hrl-kdl/hrl_geom/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
