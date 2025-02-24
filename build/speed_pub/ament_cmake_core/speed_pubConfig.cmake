# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_speed_pub_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED speed_pub_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(speed_pub_FOUND FALSE)
  elseif(NOT speed_pub_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(speed_pub_FOUND FALSE)
  endif()
  return()
endif()
set(_speed_pub_CONFIG_INCLUDED TRUE)

# output package information
if(NOT speed_pub_FIND_QUIETLY)
  message(STATUS "Found speed_pub: 0.0.0 (${speed_pub_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'speed_pub' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT speed_pub_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(speed_pub_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${speed_pub_DIR}/${_extra}")
endforeach()
