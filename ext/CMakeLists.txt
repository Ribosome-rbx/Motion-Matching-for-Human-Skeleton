# FetchContent command is available with cmake >= 3.11
include(FetchContent)

# -----------------------------------------------------------------------------
# helper funtion to download a 'fetch content declare'
# -----------------------------------------------------------------------------

# download project and add it to build tree
macro(fetch_and_add what)
    FetchContent_GetProperties("${what}")
    if (NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
        add_subdirectory(${${what}_SOURCE_DIR} ${what})
    endif ()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()

# -----------------------------------------------------------------------------
# dependencies
# -----------------------------------------------------------------------------
# crl-basic
FetchContent_Declare(
        motion-matching #
        GIT_REPOSITORY https://github.com/guo-han/motion-matching.git # https://github.com/Digital-Humans-23/motion-matching.git #
        GIT_TAG main #
)
set(MOTION_MATCHING_BUILD_APPS OFF CACHE BOOL "Build crl-basic lib only")
fetch_and_add(motion-matching)