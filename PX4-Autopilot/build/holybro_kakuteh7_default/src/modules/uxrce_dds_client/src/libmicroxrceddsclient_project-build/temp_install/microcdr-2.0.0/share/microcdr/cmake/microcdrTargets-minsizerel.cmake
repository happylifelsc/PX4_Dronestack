#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "microcdr" for configuration "MinSizeRel"
set_property(TARGET microcdr APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(microcdr PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "C"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/libmicrocdr.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS microcdr )
list(APPEND _IMPORT_CHECK_FILES_FOR_microcdr "${_IMPORT_PREFIX}/lib/libmicrocdr.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
