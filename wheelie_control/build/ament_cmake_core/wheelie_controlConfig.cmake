# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wheelie_control_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wheelie_control_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wheelie_control_FOUND FALSE)
  elseif(NOT wheelie_control_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wheelie_control_FOUND FALSE)
  endif()
  return()
endif()
set(_wheelie_control_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wheelie_control_FIND_QUIETLY)
  message(STATUS "Found wheelie_control: 0.0.0 (${wheelie_control_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wheelie_control' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${wheelie_control_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wheelie_control_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${wheelie_control_DIR}/${_extra}")
endforeach()