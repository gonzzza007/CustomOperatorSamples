cmake_minimum_required(VERSION 3.10)
project(OpenCascade) # the output name of the plugin

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

# OpenCascade (OCCT), linked dynamically. On macOS: brew install opencascade
# (on Windows point OpenCASCADE_DIR to the folder with OpenCASCADEConfig.cmake)
find_package(OpenCASCADE REQUIRED)
include_directories(${OpenCASCADE_INCLUDE_DIR})

set(OCCT_LIBS
	TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo
	TKTopAlgo TKPrim TKShHealing TKBO TKBool TKFillet TKMesh
)

BuiltCustomOp(${PROJECT_NAME} "${PROJECT_NAME}SOP.cpp" "${OCCT_LIBS}") # the name of the entry file

# let the plugin find the OCCT libraries at runtime
if (APPLE)
	set_target_properties(${PROJECT_NAME} PROPERTIES BUILD_RPATH "${OpenCASCADE_LIBRARY_DIR}")
endif()
