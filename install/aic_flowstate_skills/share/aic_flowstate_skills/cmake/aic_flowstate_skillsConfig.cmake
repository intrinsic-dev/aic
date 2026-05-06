# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_aic_flowstate_skills_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED aic_flowstate_skills_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(aic_flowstate_skills_FOUND FALSE)
  elseif(NOT aic_flowstate_skills_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(aic_flowstate_skills_FOUND FALSE)
  endif()
  return()
endif()
set(_aic_flowstate_skills_CONFIG_INCLUDED TRUE)

# output package information
if(NOT aic_flowstate_skills_FIND_QUIETLY)
  message(STATUS "Found aic_flowstate_skills: 0.0.1 (${aic_flowstate_skills_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'aic_flowstate_skills' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT aic_flowstate_skills_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(aic_flowstate_skills_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${aic_flowstate_skills_DIR}/${_extra}")
endforeach()
