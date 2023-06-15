# we need cmake >= 3.11 to support fetch_content that downloads dependencies
cmake_minimum_required(VERSION 3.11)

# set name of the project
project(motion-matching-demo)

set(CMAKE_CXX_STANDARD 17)

# release as a default build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
            "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)


# data folder path
set(MOTION_MATCHING_DEMO_DATA_FOLDER
        "\"${CMAKE_CURRENT_LIST_DIR}/data\""
        CACHE STRING ""
        )

# -----------------------------------------------------------------------------
# options
# -----------------------------------------------------------------------------

option(MOTION_MATCHING_DEMO_BUILD_APP "Build motion-matching-demo example apps." ON)

# -----------------------------------------------------------------------------
# macro and functions
# -----------------------------------------------------------------------------

# create static library named LIB_NAME
function(
        create_crl_library #
        LIB_NAME #
        SOURCE #
        DEPENDENCY #
        INCLUDE_DIRS #
        LINK_LIBS #
        COMPILE_DEFINITIONS #
)
    add_library(${LIB_NAME} STATIC ${SOURCE})
    add_library(crl::${LIB_NAME} ALIAS ${LIB_NAME})
    add_dependencies(${LIB_NAME} ${DEPENDENCY})
    target_include_directories(${LIB_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} ${LINK_LIBS})

    if (COMPILE_DEFINITIONS)
        # scope is PUBLIC so these definitions are propagated
        target_compile_definitions(${LIB_NAME} ${COMPILE_DEFINITIONS})
    endif ()

    # For solution explorer in visual studios
    set_property(TARGET ${LIB_NAME} PROPERTY FOLDER "Libs")
endfunction()

# create executable named APP_NAME
function(
        create_crl_app #
        APP_NAME #
        SOURCE #
        DEPENDENCY #
        INCLUDE_DIRS #
        LINK_LIBS #
        COMPILE_DEFINITIONS #
)

    add_executable(${APP_NAME} ${SOURCE})
    add_dependencies(${APP_NAME} ${DEPENDENCY})
    target_include_directories(${APP_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${APP_NAME} ${LINK_LIBS})

    if (COMPILE_DEFINITIONS)
        target_compile_definitions(${APP_NAME} ${COMPILE_DEFINITIONS})
    endif ()

    # For solution explorer in visual studios
    set_property(TARGET ${APP_NAME} PROPERTY FOLDER "Apps")
endfunction()

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# thirdparty code
add_subdirectory(ext)

# our code
add_subdirectory(src)

# -----------------------------------------------------------------------------
# clean up solutions view
# -----------------------------------------------------------------------------
if (WIN32)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    set_target_properties(glfw PROPERTIES FOLDER Ext)
    set_target_properties(glad PROPERTIES FOLDER Ext)
    set_target_properties(imgui PROPERTIES FOLDER Ext)
    set_target_properties(tinyobjloader PROPERTIES FOLDER Ext)
    # set_target_properties(DCA PROPERTIES FOLDER Ext)
    set_target_properties(utils PROPERTIES FOLDER Ext)
    set_target_properties(gui PROPERTIES FOLDER Ext)
    # set_target_properties(tinyxml2 PROPERTIES FOLDER Ext)
endif (WIN32)
