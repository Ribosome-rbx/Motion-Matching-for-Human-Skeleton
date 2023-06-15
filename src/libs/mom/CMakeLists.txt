cmake_minimum_required(VERSION 3.11)

project(mom)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# basic dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "crl::utils" #
        "crl::gui" #
        "crl::mocap" #
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
        )

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC
        "crl::utils" #
        PUBLIC
        "crl::gui" #
        PUBLIC
        "crl::mocap" #
        PUBLIC
        "eigen" #
)


# we use filesystem which is "experimental" for some compilers for those we need
# to add fs lib to link libraries
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # gcc
    # ~~~
    # gcc < 7: supports fs as experimentals
    # gcc = 8: supports fs but needs to link stdc++fs
    # gcc > 9: supports fs as std library
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
        list(APPEND CRL_TARGET_LINK_LIBS PUBLIC "stdc++fs")
    endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # msvc
    # ~~~
    # TODO: version check
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # llvm clang
    # ~~~
    # llvm clang needs to link stdc++fs
    list(APPEND CRL_TARGET_LINK_LIBS PUBLIC "stdc++fs")
endif ()

# create target
create_crl_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}"
)