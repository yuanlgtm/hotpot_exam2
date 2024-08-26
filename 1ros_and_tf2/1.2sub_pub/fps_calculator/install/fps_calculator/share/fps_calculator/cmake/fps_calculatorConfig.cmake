# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fps_calculator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fps_calculator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fps_calculator_FOUND FALSE)
  elseif(NOT fps_calculator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fps_calculator_FOUND FALSE)
  endif()
  return()
endif()
set(_fps_calculator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fps_calculator_FIND_QUIETLY)
  message(STATUS "Found fps_calculator: 0.0.0 (${fps_calculator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fps_calculator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fps_calculator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fps_calculator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fps_calculator_DIR}/${_extra}")
endforeach()
