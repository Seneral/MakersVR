cmake_minimum_required(VERSION 3.0)
project(MakersVR_MarkerDetector)

# Assure either debug or release is set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set build flags for debug and release
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror -Wall -std=gnu17")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wno-sign-compare -Wno-unused-label -Wno-unused-variable  -std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Switch between optimizations for Zero or 3B+
add_compile_options(-mcpu=arm1176jzf-s -mfpu=vfp)
#add_compile_options(-march=armv8-a+crc -mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mtune=cortex-a53 -mneon-for-64bits)

# Make sure all libraries are present
find_library(LIB_BCMH NAMES bcm_host HINTS /opt/vc/lib/)
find_library(LIB_VCOS NAMES vcos HINTS /opt/vc/lib/)
find_library(LIB_VCSM NAMES vcsm HINTS /opt/vc/lib/)
find_library(LIB_MMAL NAMES mmal HINTS /opt/vc/lib/)
find_library(LIB_MMAL_CORE NAMES mmal_core HINTS /opt/vc/lib/)
find_library(LIB_MMAL_UTIL NAMES mmal_util HINTS /opt/vc/lib/)
find_library(LIB_MMAL_COMP NAMES mmal_components HINTS /opt/vc/lib/)

# Set program resources
set(INCLUDES
	/usr/local/include
	/opt/vc/include
)
set(SOURCES
	source/camera/gcs.c
	source/qpu/mailbox.c
	source/qpu/qpu_base.c
	source/qpu/qpu_program.c
	source/qpu/qpu_info.c
	source/fbUtil.c
	source/blobdetection.cpp
	source/main_detector.cpp
)
set(LIBRARIES
	m dl pthread
	${LIB_BCMH} ${LIB_VCOS} ${LIB_VCSM}
	${LIB_MMAL} ${LIB_MMAL_CORE} ${LIB_MMAL_UTIL} ${LIB_MMAL_COMP}
)

# Setup build target
add_executable(Detector ${SOURCES})
target_link_libraries(Detector ${LIBRARIES})
target_include_directories(Detector PRIVATE ${INCLUDES})

# All ASM source files to build
set(QPU_PROGRAM_SOURCES
	qpu_programs/qpu_blob_tiled_min.asm
	qpu_programs/qpu_copy.asm
)

# Setup VC4ASM build target to compile all ASM source files
add_custom_target(qpu_programs)
foreach (file ${QPU_PROGRAM_SOURCES})
    get_filename_component(file_name ${file} NAME_WE)
    add_custom_command(
		OUTPUT ${file_name}.bin
		DEPENDS ${file}
	    COMMAND vc4asm -V -o ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.bin ${CMAKE_SOURCE_DIR}/${file}
	)
    add_custom_target(${file_name}
		DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.bin
	)
	add_dependencies(qpu_programs ${file_name})
endforeach()

# Make sure they are build by default
add_dependencies(Detector qpu_programs)
