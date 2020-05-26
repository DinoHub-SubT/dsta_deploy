# ignore warning for the following policies for CMake 3+
if(NOT CMAKE_MAJOR_VERSION LESS 3)
  cmake_policy(SET CMP0040 OLD)
  cmake_policy(SET CMP0046 OLD)
  cmake_policy(SET CMP0048 OLD)
endif()

# // /////////////////////////////////////////////////////////////////////////////
# // POST BUILD COMMAND: append the launch param bash script
#   -- add and source the launch param bash script to the devel/setup.bash and devel/setup.zsh
# // /////////////////////////////////////////////////////////////////////////////
function(SOURCE_LAUNCH_PARAMS)
  set(oneValueArgs TARGET SETUP_PATH)
  set(multiValueArgs ENV)
  cmake_parse_arguments(SOURCE_LAUNCH_PARAMS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  # append launch source, for every given setup script.
  foreach(env_setup_file ${SOURCE_LAUNCH_PARAMS_ENV})

    # post build, add command to append source file with launch params to setup.bash.
    add_custom_command(TARGET ${SOURCE_LAUNCH_PARAMS_TARGET} POST_BUILD
      # force calling this target on every build, but 'touch' updating dummy.cpp
      # -- needs to be first in case the execution fails
      COMMAND touch ${PROJECT_SOURCE_DIR}/src/dummy.cpp

      # append the bash params to the devel/setup.bash (or similar bash env suffix)
      COMMAND ${PROJECT_SOURCE_DIR}/scripts/subt-params-install.bash --install "${SOURCE_LAUNCH_PARAMS_SETUP_PATH}/${env_setup_file}"

      # get the result of the command
      DEPENDS always_rebuild
      RESULT_VARIABLE retcode
      ERROR_VARIABLE errorcode
    )

    # post build command execution failed, print out the error string
    if(NOT "${retcode}" STREQUAL "")
      if(NOT "${retcode}" STREQUAL "0")
        message(FATAL_ERROR "Fatal error adding the launch param bash script to the ROS devel setup.bash/zsh.")
      endif()
    endif()

  endforeach()

endfunction()