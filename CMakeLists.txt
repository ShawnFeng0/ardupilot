cmake_minimum_required(VERSION 2.8.4)
project(Ardupilot)

# Function:                 EXCLUDE_FILES_FROM_DIR_IN_LIST
# Description:              Exclude all files from a list under a specific directory.
# Param _InFileList:        Input and returned List
# Param _excludeDirName:    Name of the directory, which shall be ignored.
# Param _verbose:           Print the names of the files handled

FUNCTION (EXCLUDE_FILES_FROM_DIR_IN_LIST _InFileList _excludeDirName _verbose)
  foreach (ITR ${_InFileList})
    if ("${_verbose}")
      message(STATUS "ITR=${ITR}")
    endif ("${_verbose}")

    if ("${ITR}" MATCHES "(.*)${_excludeDirName}(.*)")                   # Check if the item matches the directory name in _excludeDirName
      message(STATUS "Remove Item from List:${ITR}")
      list (REMOVE_ITEM _InFileList ${ITR})                              # Remove the item from the list
    endif ("${ITR}" MATCHES "(.*)${_excludeDirName}(.*)")

  endforeach(ITR)
  set(SOURCE_FILES ${_InFileList} PARENT_SCOPE)                          # Return the SOURCE_FILES variable to the calling parent
ENDFUNCTION (EXCLUDE_FILES_FROM_DIR_IN_LIST)


# we just grab all the cpp and h files. not ideal, but works well enough
# means a "reload CMake project" when you add new files
file(GLOB_RECURSE SOURCE_FILES APMrover2/*.cpp APMrover2/*.h)
file(GLOB_RECURSE SOURCE_FILES ArduCopter/*.cpp ArduCopter/*.h)
file(GLOB_RECURSE SOURCE_FILES ArduPlane/*.cpp ArduPlane/*.h)
file(GLOB_RECURSE SOURCE_FILES build/sitl/libraries/GCS_MAVLink/include/*.cpp build/sitl/libraries/GCS_MAVLink/include/*.h)
file(GLOB_RECURSE SOURCE_FILES libraries/*.cpp libraries/*.h)

EXCLUDE_FILES_FROM_DIR_IN_LIST("${SOURCE_FILES}" "/libraries/AP_HAL_F4Light/support/minimosd-extra/Character_Updater/" FALSE)
EXCLUDE_FILES_FROM_DIR_IN_LIST("${SOURCE_FILES}" "/libraries/AP_HAL_F4Light/hardware/osd/osd_core/" FALSE)


include_directories(APMrover2)
include_directories(ArduCopter)
include_directories(ArduPlane)
include_directories(libraries)
include_directories(modules)
# To work with SITL 
add_definitions(-DSKETCHBOOK="./")
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_SITL)
#add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_PX4)
add_executable(fake-target-for-clion ${SOURCE_FILES})