#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros2_control_t170::ros2_control_t170" for configuration "Release"
set_property(TARGET ros2_control_t170::ros2_control_t170 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ros2_control_t170::ros2_control_t170 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libros2_control_t170.so"
  IMPORTED_SONAME_RELEASE "libros2_control_t170.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ros2_control_t170::ros2_control_t170 )
list(APPEND _IMPORT_CHECK_FILES_FOR_ros2_control_t170::ros2_control_t170 "${_IMPORT_PREFIX}/lib/libros2_control_t170.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
