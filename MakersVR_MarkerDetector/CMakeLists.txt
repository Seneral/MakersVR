cmake_minimum_required(VERSION 2.8)
project(MakersVR_MarkerDetector)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror -Wall -std=gnu99 -Wno-psabi -g")

include_directories(/usr/local/include)
include_directories(/opt/vc/include/)
include_directories(/opt/vc/include/interface/vcsm)

include_directories(./camera)
include_directories(./gl)

link_directories(/opt/vc/lib/)

find_library(LIB_BCMH NAMES bcm_host HINTS /opt/vc/lib/)
find_library(LIB_VCOS NAMES vcos HINTS /opt/vc/lib/)
find_library(LIB_VCSM NAMES vcsm HINTS /opt/vc/lib/)
find_library(LIB_MMAL NAMES mmal HINTS /opt/vc/lib/)
find_library(LIB_MMAL_CORE NAMES mmal_core HINTS /opt/vc/lib/)
find_library(LIB_MMAL_UTIL NAMES mmal_util HINTS /opt/vc/lib/)
find_library(LIB_MMAL_COMP NAMES mmal_components HINTS /opt/vc/lib/)
find_library(LIB_BEGL NAMES brcmEGL HINTS /opt/vc/lib/)
find_library(LIB_GLES NAMES brcmGLESv2 HINTS /opt/vc/lib/)

set(VC4CV_GL_SOURCES
   camera/gcs.c
   camera/camGL.c
   gl/eglUtil.c
   gl/mesh.cpp
   gl/shader.cpp
   gl/texture.cpp)

set(VC4CV_GL_LIBRARIES
	m dl pthread
	${LIB_BCMH} ${LIB_VCOS} ${LIB_VCSM}
	${LIB_MMAL} ${LIB_MMAL_CORE} ${LIB_MMAL_UTIL} ${LIB_MMAL_COMP}
	${LIB_BEGL} ${LIB_GLES})

# Makers VR MarkerDetector with GL backend
add_executable(GLDetector ${VC4CV_GL_SOURCES} main_gl_detector.cpp blobdetection.cpp)
target_link_libraries(GLDetector ${VC4CV_GL_LIBRARIES})