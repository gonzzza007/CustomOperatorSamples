cmake_minimum_required(VERSION 3.10)
project(AlphaShapesParallel) # the output name of the plugin

message(STATUS "** Building ${PROJECT_NAME} SOP **")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(COPY_TO_TD_PLUGINS "Copy the built plugin to the TouchDesigner Plugins folder" ON)

if (APPLE)
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -stdlib=libc++")
endif()


set(OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/plugin)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL ${OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUTPUT_DIRECTORY})

###

function(BuiltCustomOp OPNAME SRC LINK_LIBRARIES)

	file(GLOB PROJECT_SRC CONFIGURE_DEPENDS
		"*.h"
		"*.hpp"
		"*.cpp"
	)

	add_library(${OPNAME} MODULE ${PROJECT_SRC} ${DEP_SRC})

	set_target_properties(${OPNAME} PROPERTIES LINKER_LANGUAGE CXX)
	set_target_properties(${OPNAME} PROPERTIES PREFIX "")
	set_target_properties(${OPNAME} PROPERTIES BUNDLE TRUE)
	set_target_properties(${OPNAME} PROPERTIES BUNDLE_EXTENSION "plugin")

	target_link_libraries(${OPNAME} PRIVATE ${LINK_LIBRARIES})

	set(OPTEST ${OPNAME}_TEST)
    add_custom_target(${OPTEST})

    set_target_properties(${OPTEST} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/plugin)
    set_target_properties(${OPTEST} PROPERTIES VS_DEBUGGER_ENVIRONMENT "TOUCH_TEXT_CONSOLE=1")

	# linked dylibs resolved from the host app instead (see INSTALL_NAME_CHANGES below)
	if (APPLE AND INSTALL_NAME_CHANGES)
		add_custom_command(TARGET ${OPNAME} POST_BUILD
			COMMAND install_name_tool ${INSTALL_NAME_CHANGES} "$<TARGET_FILE:${OPNAME}>"
			COMMAND codesign --force --sign - "$<TARGET_BUNDLE_DIR:${OPNAME}>"
			COMMENT "Linking ${OPNAME} against the host app's dylibs")
	endif()

	if (COPY_TO_TD_PLUGINS AND APPLE)
		set(TD_PLUGINS_DIR "$ENV{HOME}/Library/Application Support/Derivative/TouchDesigner099/Plugins")
		add_custom_command(TARGET ${OPNAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E make_directory "${TD_PLUGINS_DIR}"
			COMMAND ${CMAKE_COMMAND} -E rm -rf "${TD_PLUGINS_DIR}/${OPNAME}.plugin"
			COMMAND ${CMAKE_COMMAND} -E copy_directory "$<TARGET_BUNDLE_DIR:${OPNAME}>" "${TD_PLUGINS_DIR}/${OPNAME}.plugin"
			COMMENT "Copying ${OPNAME}.plugin to ${TD_PLUGINS_DIR}")
	elseif (COPY_TO_TD_PLUGINS AND WIN32)
		file(TO_CMAKE_PATH "$ENV{USERPROFILE}/Documents/Derivative/Plugins" TD_PLUGINS_DIR)
		# the DLLs of linked libraries (e.g. OCCT) have to be next to the plugin DLL
		set(RUNTIME_DLLS "")
		if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.21)
			set(RUNTIME_DLLS "$<TARGET_RUNTIME_DLLS:${OPNAME}>")
		endif()
		add_custom_command(TARGET ${OPNAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E make_directory "${TD_PLUGINS_DIR}"
			COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:${OPNAME}>" ${RUNTIME_DLLS} "${TD_PLUGINS_DIR}"
			COMMAND_EXPAND_LISTS
			COMMENT "Copying ${OPNAME}.dll to ${TD_PLUGINS_DIR}")
	endif()

endfunction(BuiltCustomOp)

###

# CGAL (header-only) and its dependencies (Boost, GMP, MPFR), on macOS: brew install cgal

# Link GMP and MPFR statically so the plugin doesn't need Homebrew at runtime
# (same idea as the x64-windows-static-md vcpkg triplet on Windows).
# CGAL looks GMP up through several cache variables, so all of them are pointed
# at the static archives before find_package(CGAL).
option(STATIC_CGAL_DEPS "Link GMP and MPFR statically" ON)
set(STATIC_DEP_VARS GMP_LIBRARIES GMP_LIBRARY_RELEASE GMP_LIBRARY_DEBUG GMPXX_LIBRARIES MPFR_LIBRARIES)

if (APPLE AND STATIC_CGAL_DEPS)
	find_library(GMP_STATIC_LIB NAMES libgmp.a HINTS /opt/homebrew/opt/gmp/lib /usr/local/opt/gmp/lib)
	find_library(GMPXX_STATIC_LIB NAMES libgmpxx.a HINTS /opt/homebrew/opt/gmp/lib /usr/local/opt/gmp/lib)
	find_library(MPFR_STATIC_LIB NAMES libmpfr.a HINTS /opt/homebrew/opt/mpfr/lib /usr/local/opt/mpfr/lib)

	if (GMP_STATIC_LIB AND GMPXX_STATIC_LIB AND MPFR_STATIC_LIB)
		message(STATUS "Linking GMP and MPFR statically")
		set(GMP_LIBRARIES ${GMP_STATIC_LIB} CACHE FILEPATH "" FORCE)
		set(GMP_LIBRARY_RELEASE ${GMP_STATIC_LIB} CACHE FILEPATH "" FORCE)
		set(GMP_LIBRARY_DEBUG ${GMP_STATIC_LIB} CACHE FILEPATH "" FORCE)
		set(GMPXX_LIBRARIES ${GMPXX_STATIC_LIB} CACHE FILEPATH "" FORCE)
		set(MPFR_LIBRARIES ${MPFR_STATIC_LIB} CACHE FILEPATH "" FORCE)
	else()
		message(WARNING "Static GMP/MPFR not found, linking dynamically (brew install gmp mpfr)")
	endif()
else()
	# drop static archives cached by a previous STATIC_CGAL_DEPS=ON configure
	foreach(VAR ${STATIC_DEP_VARS})
		if ("${${VAR}}" MATCHES "\\.a$")
			unset(${VAR} CACHE)
		endif()
	endforeach()
endif()

find_package(CGAL REQUIRED)

set(LINK_LIBS "CGAL::CGAL")

# Parallel Delaunay triangulation with TBB, on macOS: brew install tbb
option(USE_TBB "Build the triangulation in parallel with TBB" ON)
if (USE_TBB)
	find_package(TBB)
	include(CGAL_TBB_support)
	if (TARGET CGAL::TBB_support)
		message(STATUS "Using TBB ${TBB_VERSION}")
		list(APPEND LINK_LIBS CGAL::TBB_support)
	else()
		message(WARNING "TBB not found, building sequential (brew install tbb)")
	endif()
endif()

# TouchDesigner ships TBB in its Frameworks folder (@rpath/libtbb.12.dylib): the plugin
# loads that one instead of the Homebrew one, so there is a single TBB in the process
# and no Homebrew dependency at runtime. BuiltCustomOp() rewrites the install names.
set(INSTALL_NAME_CHANGES "")
if (APPLE AND TARGET CGAL::TBB_support)
	foreach(TBB_TARGET TBB::tbb TBB::tbbmalloc)
		get_target_property(TBB_LIB ${TBB_TARGET} IMPORTED_LOCATION_RELEASE)
		if (NOT TBB_LIB)
			get_target_property(TBB_LIB ${TBB_TARGET} IMPORTED_LOCATION)
		endif()
		execute_process(COMMAND otool -D "${TBB_LIB}" OUTPUT_VARIABLE TBB_ID OUTPUT_STRIP_TRAILING_WHITESPACE)
		string(REGEX REPLACE "^[^\n]*\n" "" TBB_ID "${TBB_ID}")
		get_filename_component(TBB_NAME "${TBB_ID}" NAME)
		list(APPEND INSTALL_NAME_CHANGES -change "${TBB_ID}" "@rpath/${TBB_NAME}")
	endforeach()
endif()

BuiltCustomOp(${PROJECT_NAME} "${PROJECT_NAME}SOP.cpp" "${LINK_LIBS}") # the name of the entry file
