CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
IF (COMMAND CMAKE_POLICY)
    IF (POLICY CMP0003)
        CMAKE_POLICY (SET CMP0003 NEW)
    ENDIF (POLICY CMP0003)
    IF (POLICY CMP0011)
        CMAKE_POLICY (SET CMP0011 NEW)
    ENDIF (POLICY CMP0011)
ENDIF (COMMAND CMAKE_POLICY)

INCLUDE (PlayerUtils)

INCLUDE (FindPkgConfig)
IF (NOT PKG_CONFIG_FOUND)
    SET (PLUGIN_PLAYERC_CFLAGS "")
    SET (PLUGIN_PLAYERC_INCLUDE_DIRS )
    LIST (APPEND PLUGIN_PLAYERC_INCLUDE_DIRS "/usr/local/dcs/versions/player-3.0.2/include/player-3.0")
    SET (PLUGIN_PLAYERC_LINK_LIBS m;z)
    LIST (APPEND PLUGIN_PLAYERC_LINK_LIBS "playerc")
    SET (PLUGIN_PLAYERC_LIBRARY_DIRS )
    LIST (APPEND PLUGIN_PLAYERC_LIBRARY_DIRS "/usr/local/dcs/versions/player-3.0.2/lib")
    SET (PLUGIN_PLAYERC_LINK_FLAGS "")
ELSE (NOT PKG_CONFIG_FOUND)
    pkg_check_modules (PLUGIN_PLAYERC_PKG REQUIRED playerc)
    IF (PLUGIN_PLAYERC_PKG_CFLAGS_OTHER)
        LIST_TO_STRING (PLUGIN_PLAYERC_CFLAGS "${PLUGIN_PLAYERC_PKG_CFLAGS_OTHER}")
    ELSE (PLUGIN_PLAYERC_PKG_CFLAGS_OTHER)
        SET (PLUGIN_PLAYERC_CFLAGS_OTHER "")
    ENDIF (PLUGIN_PLAYERC_PKG_CFLAGS_OTHER)
    SET (PLUGIN_PLAYERC_INCLUDE_DIRS ${PLUGIN_PLAYERC_PKG_INCLUDE_DIRS})
    SET (PLUGIN_PLAYERC_LINK_LIBS ${PLUGIN_PLAYERC_PKG_LIBRARIES})
    SET (PLUGIN_PLAYERC_LIBRARY_DIRS ${PLUGIN_PLAYERC_PKG_LIBRARY_DIRS})
    IF (PLUGIN_PLAYERC_PKG_LDFLAGS_OTHER)
        LIST_TO_STRING (PLUGIN_PLAYERC_LINK_FLAGS ${PLUGIN_PLAYERC_PKG_LDFLAGS_OTHER})
    ELSE (PLUGIN_PLAYERC_PKG_LDFLAGS_OTHER)
        SET (PLUGIN_PLAYERC_LINK_FLAGS "")
    ENDIF (PLUGIN_PLAYERC_PKG_LDFLAGS_OTHER)
ENDIF (NOT PKG_CONFIG_FOUND)

INCLUDE (FindPkgConfig)
IF (NOT PKG_CONFIG_FOUND)
    SET (PLAYERCORE_CFLAGS "")
    SET (PLAYERCORE_INCLUDE_DIRS )
    LIST (APPEND PLAYERCORE_INCLUDE_DIRS "/usr/local/dcs/versions/player-3.0.2/include/player-3.0")
    SET (PLAYERCORE_LINK_LIBS pthread;ltdl;dl)
    LIST (APPEND PLAYERCORE_LINK_LIBS "playercore")
    SET (PLAYERCORE_LIBRARY_DIRS )
    LIST (APPEND PLAYERCORE_LIBRARY_DIRS "/usr/local/dcs/versions/player-3.0.2/lib")
    SET (PLAYERCORE_LINK_FLAGS "")
ELSE (NOT PKG_CONFIG_FOUND)
    pkg_check_modules (PLAYERCORE_PKG REQUIRED playercore)
    IF (PLAYERCORE_PKG_CFLAGS_OTHER)
        LIST_TO_STRING (PLAYERCORE_CFLAGS "${PLAYERCORE_PKG_CFLAGS_OTHER}")
    ELSE (PLAYERCORE_PKG_CFLAGS_OTHER)
        SET (PLAYERCORE_CFLAGS "")
    ENDIF (PLAYERCORE_PKG_CFLAGS_OTHER)
    SET (PLAYERCORE_INCLUDE_DIRS ${PLAYERCORE_PKG_INCLUDE_DIRS})
    SET (PLAYERCORE_LINK_LIBS ${PLAYERCORE_PKG_LIBRARIES})
    SET (PLAYERCORE_LIBRARY_DIRS ${PLAYERCORE_PKG_LIBRARY_DIRS})
    IF (PLAYERCORE_PKG_LDFLAGS_OTHER)
        LIST_TO_STRING (PLAYERCORE_LINK_FLAGS ${PLAYERCORE_PKG_LDFLAGS_OTHER})
    ELSE (PLAYERCORE_PKG_LDFLAGS_OTHER)
        SET (PLAYERCORE_LINK_FLAGS "")
    ENDIF (PLAYERCORE_PKG_LDFLAGS_OTHER)
ENDIF (NOT PKG_CONFIG_FOUND)


# This is slightly different from the one used by the Player build system itself.
# It takes a single file instead of a directory. It also expects playerinterfacegen.py
# to have been installed in the system path (as it should have been with Player).
MACRO (PROCESS_INTERFACES _file _outputFile)
    ADD_CUSTOM_COMMAND (OUTPUT ${_outputFile}
        COMMAND playerinterfacegen ${ARGN} ${_file} > ${_outputFile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${_file}
    )
ENDMACRO (PROCESS_INTERFACES)

MACRO (PROCESS_XDR _interfaceH _xdrH _xdrC)
    ADD_CUSTOM_COMMAND (OUTPUT ${_xdrH} ${_xdrC}
        COMMAND playerxdrgen ${_interfaceH} ${_xdrC} ${_xdrH}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${_interfaceH})
ENDMACRO (PROCESS_XDR)


###############################################################################
# Macro to build a plugin driver.
# _driverName: The name of the driver library to create
# Pass source files, flags, etc. as extra args preceded by keywords as follows:
# SOURCES <source file list>
# INCLUDEDIRS <include directories list>
# LIBDIRS <library directories list>
# LINKFLAGS <link flags list>
# CFLAGS <compile flags list>
# See the examples directory (typically, ${prefix}/share/player/examples) for
# example CMakeLists.txt files.
MACRO (PLAYER_ADD_PLUGIN_DRIVER _driverName)
    PLAYER_PROCESS_ARGUMENTS (_srcs _includeDirs _libDirs _linkLibs _linkFlags _cFlags _junk ${ARGN})
    IF (_junk)
        MESSAGE (STATUS "WARNING: unkeyworded arguments found in PLAYER_ADD_PLUGIN_DRIVER: ${_junk}")
    ENDIF (_junk)
    LIST_TO_STRING (_cFlags "${_cFlags}")

    IF (_includeDirs OR PLAYERCORE_INCLUDE_DIRS)
        INCLUDE_DIRECTORIES (${_includeDirs} ${PLAYERCORE_INCLUDE_DIRS})
    ENDIF (_includeDirs OR PLAYERCORE_INCLUDE_DIRS)
    IF (_libDirs OR PLAYERCORE_LIBRARY_DIRS)
        LINK_DIRECTORIES (${_libDirs} ${PLAYERCORE_LIBRARY_DIRS})
    ENDIF (_libDirs OR PLAYERCORE_LIBRARY_DIRS)

    ADD_LIBRARY (${_driverName} SHARED ${_srcs})
    IF (PLAYERCORE_LIBRARY_DIRS)
        SET_TARGET_PROPERTIES (${_driverName} PROPERTIES
            INSTALL_RPATH ${PLAYERCORE_LIBRARY_DIRS}
            BUILD_WITH_INSTALL_RPATH TRUE)
    ENDIF (PLAYERCORE_LIBRARY_DIRS)
    IF (_linkFlags)
        LIST_TO_STRING (_linkFlags "${_linkFlags}")
        SET_TARGET_PROPERTIES (${_driverName} PROPERTIES LINK_FLAGS ${_linkFlags})
    ENDIF (_linkFlags)
    IF (PLAYERCORE_LINK_FLAGS)
        SET_TARGET_PROPERTIES (${_driverName} PROPERTIES LINK_FLAGS ${PLAYERCORE_LINK_FLAGS})
    ENDIF (PLAYERCORE_LINK_FLAGS)
    IF (_linkLibs)
        TARGET_LINK_LIBRARIES (${_driverName} ${_linkLibs})
    ENDIF (_linkLibs)
    IF (PLAYERCORE_LINK_LIBS)
        TARGET_LINK_LIBRARIES (${_driverName} ${PLAYERCORE_LINK_LIBS})
    ENDIF (PLAYERCORE_LINK_LIBS)

    # Get the current cflags for each source file, and add the global ones
    # (this allows the user to specify individual cflags for each source file
    # without the global ones overriding them).
    IF (PLAYERCORE_CFLAGS OR _cFLags)
        FOREACH (_file ${_srcs})
            GET_SOURCE_FILE_PROPERTY (_fileCFlags ${_file} COMPILE_FLAGS)
            IF (_fileCFlags STREQUAL NOTFOUND)
                SET (_newCFlags "${PLAYERCORE_CFLAGS} ${_cFlags}")
            ELSE (_fileCFlags STREQUAL NOTFOUND)
                SET (_newCFlags "${_fileCFlags} ${PLAYERCORE_CFLAGS} ${_cFlags}")
            ENDIF (_fileCFlags STREQUAL NOTFOUND)
            SET_SOURCE_FILES_PROPERTIES (${_file} PROPERTIES
                COMPILE_FLAGS ${_newCFlags})
        ENDFOREACH (_file)
    ENDIF (PLAYERCORE_CFLAGS OR _cFLags)
ENDMACRO (PLAYER_ADD_PLUGIN_DRIVER)


###############################################################################
# Macro to build a plugin interface.
# This macro will create generated sources prefixed with the interface name
# _interfName: The name of the interface library (not the interface itself!)
#              to create
# _interfDef: The interface definition file
#
# Pass source files, flags, etc. as extra args preceded by keywords as follows:
# SOURCES <source file list>
# INCLUDEDIRS <include directories list>
# LIBDIRS <library directories list>
# LINKFLAGS <link flags list>
# CFLAGS <compile flags list>
# See the examples directory (typically, ${prefix}/share/player/examples) for
# example CMakeLists.txt files.
INCLUDE (FindPythonInterp)
MACRO (PLAYER_ADD_PLUGIN_INTERFACE _interfName _interfDef)
    IF (NOT PYTHONINTERP_FOUND)
        MESSAGE (FATAL_ERROR "No Python interpreter found. Cannot continue.")
    ENDIF (NOT PYTHONINTERP_FOUND)

    PLAYER_PROCESS_ARGUMENTS (_srcs _includeDirs _libDirs _linkLibs _linkFlags _cFlags _junk ${ARGN})
    IF (_junk)
        MESSAGE (STATUS
                "WARNING: unkeyworded arguments found in PLAYER_ADD_PLUGIN_INTERFACE: ${_junk}")
    ENDIF (_junk)
    LIST_TO_STRING (_cFlags "${_cFlags}")

    IF (_includeDirs OR PLUGIN_PLAYERC_INCLUDE_DIRS)
        INCLUDE_DIRECTORIES (${_includeDirs} ${PLUGIN_PLAYERC_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
    ENDIF (_includeDirs OR PLUGIN_PLAYERC_INCLUDE_DIRS)
    IF (_libDirs OR PLUGIN_PLAYERC_LIBRARY_DIRS)
        LINK_DIRECTORIES (${_libDirs} ${PLUGIN_PLAYERC_LIBRARY_DIRS})
    ENDIF (_libDirs OR PLUGIN_PLAYERC_LIBRARY_DIRS)

    # Have to generate some source files
    SET (interface_h ${CMAKE_CURRENT_BINARY_DIR}/${_interfName}_interface.h)
    PROCESS_INTERFACES (${_interfDef} ${interface_h} --plugin)
    SET (functiontable_c ${CMAKE_CURRENT_BINARY_DIR}/${_interfName}_functiontable.c)
    PROCESS_INTERFACES (${_interfDef} ${functiontable_c} --plugin --functiontable)
    IF (PLUGIN_PLAYERC_CFLAGS)
        SET_SOURCE_FILES_PROPERTIES (${functiontable_c} PROPERTIES
                                     COMPILE_FLAGS ${PLUGIN_PLAYERC_CFLAGS})
    ENDIF (PLUGIN_PLAYERC_CFLAGS)
    SET (xdr_h ${CMAKE_CURRENT_BINARY_DIR}/${_interfName}_xdr.h)
    SET (xdr_c ${CMAKE_CURRENT_BINARY_DIR}/${_interfName}_xdr.c)
    PROCESS_XDR (${interface_h} ${xdr_h} ${xdr_c})
    IF (PLUGIN_PLAYERC_CFLAGS)
        SET_SOURCE_FILES_PROPERTIES (${xdr_c} PROPERTIES COMPILE_FLAGS ${PLUGIN_PLAYERC_CFLAGS})
    ENDIF (PLUGIN_PLAYERC_CFLAGS)

    ADD_LIBRARY (${_interfName} SHARED ${interface_h} ${functiontable_c} ${xdr_h} ${xdr_c} ${_srcs})
    IF (PLUGIN_PLAYERC_LIBRARY_DIRS)
        SET_TARGET_PROPERTIES (${_interfName} PROPERTIES
            INSTALL_RPATH ${PLUGIN_PLAYERC_LIBRARY_DIRS}
            BUILD_WITH_INSTALL_RPATH TRUE)
    ENDIF (PLUGIN_PLAYERC_LIBRARY_DIRS)
    IF (_linkFlags)
        SET_TARGET_PROPERTIES (${_interfName} PROPERTIES LINK_FLAGS ${_linkFlags})
    ENDIF (_linkFlags)
    IF (PLUGIN_PLAYERC_LINK_FLAGS)
        SET_TARGET_PROPERTIES (${_interfName} PROPERTIES LINK_FLAGS ${PLUGIN_PLAYERC_LINK_FLAGS})
    ENDIF (PLUGIN_PLAYERC_LINK_FLAGS)
    IF (_linkLibs)
        TARGET_LINK_LIBRARIES (${_interfName} ${_linkLibs})
    ENDIF (_linkLibs)
    IF (PLUGIN_PLAYERC_LINK_LIBS)
        TARGET_LINK_LIBRARIES (${_interfName} ${PLUGIN_PLAYERC_LINK_LIBS})
    ENDIF (PLUGIN_PLAYERC_LINK_LIBS)
    GET_SOURCE_FILE_PROPERTY (currentCFlags ${xdr_c} COMPILE_FLAGS)
    IF (currentCFlags STREQUAL "NOTFOUND")
        SET (newCFlags "-Dplayerxdr_EXPORTS")
    ELSE (currentCFlags STREQUAL "NOTFOUND")
        SET (newCFlags "-Dplayerxdr_EXPORTS ${currentCFlags}")
    ENDIF (currentCFlags STREQUAL "NOTFOUND")
    SET_SOURCE_FILES_PROPERTIES (${xdr_c} PROPERTIES COMPILE_FLAGS ${newCFlags})

    # Get the current cflags for each source file, and add the global ones
    # (this allows the user to specify individual cflags for each source file
    # without the global ones overriding them).
    IF (PLUGIN_PLAYERC_CFLAGS OR _cFLags)
        FOREACH (_file ${_srcs})
            GET_SOURCE_FILE_PROPERTY (_fileCFlags ${_file} COMPILE_FLAGS)
            IF (_fileCFlags STREQUAL NOTFOUND)
                SET (_newCFlags "${PLUGIN_PLAYERC_CFLAGS} ${_cFlags}")
            ELSE (_fileCFlags STREQUAL NOTFOUND)
                SET (_newCFlags "${_fileCFlags} ${PLUGIN_PLAYERC_CFLAGS} ${_cFlags}")
            ENDIF (_fileCFlags STREQUAL NOTFOUND)
            SET_SOURCE_FILES_PROPERTIES (${_file} PROPERTIES
                COMPILE_FLAGS ${_newCFlags})
        ENDFOREACH (_file)
    ENDIF (PLUGIN_PLAYERC_CFLAGS OR _cFLags)
ENDMACRO (PLAYER_ADD_PLUGIN_INTERFACE)
