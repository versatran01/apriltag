# FindOgre.cmake - Find Ogre
# quchao@seas.upenn.edu (Chao Qu)
# Modified from FindGlog.cmake by alexs.mac@gmail.com (Alex Stewart)
#
# FindOgre.cmake - Find Ogre library.
#
# This module defines the following variables:
#
# Ogre_FOUND:        TRUE iff Ogre is found.
# Ogre_INCLUDE_DIRS: Include directories for Ogre.
# Ogre_LIBRARIES:    Libraries required to link Ogre.
#
# The following variables control the behaviour of this module:
#
# Ogre_INCLUDE_DIR_HINTS: List of additional directories in which to
#                         search for Ogre includes, e.g: /foo/include.
# Ogre_LIBRARY_DIR_HINTS: List of additional directories in which to
#                         search for Ogre libraries, e.g: /bar/lib.
#
# The following variables are also defined by this module, but in line with
# CMake recommended FindPackage() module style should NOT be referenced directly
# by callers (use the plural variables detailed above instead).  These variables
# do however affect the behaviour of the module via FIND_[PATH/LIBRARY]() which
# are NOT re-called (i.e. search for library is not repeated) if these variables
# are set with valid values _in the CMake cache_. This means that if these
# variables are set directly in the cache, either by the user in the CMake GUI,
# or by the user passing -DVAR=VALUE directives to CMake when called (which
# explicitly defines a cache variable), then they will be used verbatim,
# bypassing the HINTS variables and other hard-coded search locations.
#
# Ogre_INCLUDE_DIR: Include directory for Ogre, not including the
#                   include directory of any dependencies.
# Ogre_LIBRARY: Ogre library, not including the libraries of any
#               dependencies.

# Called if we failed to find Ogre or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(Ogre_REPORT_NOT_FOUND REASON_MSG)
    unset(Ogre_FOUND)
    unset(Ogre_INCLUDE_DIRS)
    unset(Ogre_LIBRARIES)
    # Make results of search visible in the CMake GUI if Ogre has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR Ogre_INCLUDE_DIR Ogre_LIBRARY)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Ogre_FIND_QUIETLY)
        message(STATUS "Failed to find Ogre - " ${REASON_MSG} ${ARGN})
    elseif(Ogre_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find Ogre - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find Ogre - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(Ogre_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND Ogre_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include/x86_64-linux-gnu
    /usr/include
    )
list(APPEND Ogre_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib/x86_64-linux-gnu
    /usr/lib
    )

# Check general hints
if(Ogre_HINTS AND EXISTS ${Ogre_HINTS})
    set(Ogre_INCLUDE_DIR_HINTS ${Ogre_HINTS}/include)
    set(Ogre_LIBRARY_DIR_HINTS ${Ogre_HINTS}/lib)
endif()

set(Ogre_INCLUDE_FILE OGRE/Ogre.h)
# Search supplied hint directories first if supplied.
find_path(Ogre_INCLUDE_DIR
    NAMES ${Ogre_INCLUDE_FILE}
    PATHS ${Ogre_INCLUDE_DIR_HINTS}
          ${Ogre_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT Ogre_INCLUDE_DIR OR NOT EXISTS ${Ogre_INCLUDE_DIR})
Ogre_REPORT_NOT_FOUND("Could not find Ogre include directory, "
    "set Ogre_INCLUDE_DIR to directory containing Ogre/Ogre.h")
endif()

find_library(Ogre_LIBRARY
    NAMES OgreMain
    PATHS ${Ogre_LIBRARY_DIR_HINTS}
          ${Ogre_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT Ogre_LIBRARY OR NOT EXISTS ${Ogre_LIBRARY})
Ogre_REPORT_NOT_FOUND("Could not find Ogre library, "
    "set Ogre_LIBRARY to full path to libOgreMain.so.")
else()
    string(REGEX MATCH ".*/" Ogre_LIBRARY_DIR ${Ogre_LIBRARY})
endif()

# Mark internally as found, then verify. Ogre_REPORT_NOT_FOUND() unsets
# if called.
set(Ogre_FOUND TRUE)

# Catch case when caller has set Ogre_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(Ogre_INCLUDE_DIR AND NOT EXISTS ${Ogre_INCLUDE_DIR}/${Ogre_INCLUDE_FILE})
Ogre_REPORT_NOT_FOUND("Caller defined Ogre_INCLUDE_DIR:"
    " ${Ogre_INCLUDE_DIR} does not contain Ogre/config.h header.")
endif()

# TODO: This regex for Ogre library is pretty primitive, we use lowercase
#       for comparison to handle Windows using CamelCase library names, could
#       this check be better?
string(TOLOWER "${Ogre_LIBRARY}" LOWERCASE_Ogre_LIBRARY)
if(Ogre_LIBRARY AND NOT "${LOWERCASE_Ogre_LIBRARY}" MATCHES ".*ogre*")
Ogre_REPORT_NOT_FOUND("Caller defined Ogre_LIBRARY: "
    "${Ogre_LIBRARY} does not match Ogre.")
endif()

# Set standard CMake FindPackage variables if found.
if(Ogre_FOUND)
    list(APPEND Ogre_INCLUDE_DIRS ${Ogre_INCLUDE_DIR} /usr/include/suitesparse)
    file(GLOB Ogre_LIBRARIES ${Ogre_LIBRARY_DIR}libOgre*)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Ogre DEFAULT_MSG
    Ogre_INCLUDE_DIRS Ogre_LIBRARIES)

# Only mark internal variables as advanced if we found Ogre, otherwise
# leave them visible in the standard GUI for the user to set manually.
if(Ogre_FOUND)
    mark_as_advanced(FORCE Ogre_INCLUDE_DIR Ogre_LIBRARY)
endif()
