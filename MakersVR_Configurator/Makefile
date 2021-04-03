# Set Debug options
ifdef DEBUG
cdebug = -g
ldebug =
d = d
mode = debug
else
cdebug = -O2	
ldebug =
d =
mode = release
endif

# Find library folders
libs = lib/linux/$(mode)
cvl = $(libs)/opencv2
wxl = $(libs)/wx
# An additional configuration include
wxi = $(wildcard $(wxl)/wx/include/*)

# Setup compiler environment
i = include
s = source
b = build
o = build/obj
cflags = -DGLEW_STATIC $(cdebug)
cxxflags = $(cflags) -I $(i) -I $(wxi) -DEIGEN_MPL2_ONLY -D__WXGTK__ -D_FILE_OFFSET_BITS=64 -D_UNICODE
# Generic C and CPP targets
$(o)/%.obj: $(s)/%.c Makefile
	gcc $(cflags) -c $< -o $@
$(o)/%.obj: $(s)/%.cpp Makefile
	g++ $(cxxflags) -c $< -o $@

# Define the libs to use based on debug options
wxlibs = $(wildcard $(wxl)/libwx_*u_*-3.1.a) "$(wxl)/libwx_baseu-3.1.a" "$(wxl)/libwxregexu-3.1.a"
cvlibs = $(cvl)/libopencv_calib3d.a $(cvl)/libopencv_flann.a $(cvl)/libopencv_imgproc.a $(cvl)/libopencv_features2d.a $(cvl)/libopencv_core.a
usblibs = $(wildcard $(libs)/libusb/libusb*.so)
syslibs = -lpthread -lz -lrt -lz -lGL \
	-lgtk-x11-2.0 -lgdk-x11-2.0 -lpangocairo-1.0 -latk-1.0 -lcairo -lgdk_pixbuf-2.0 -lX11 -lSM -lpangoft2-1.0 -lpango-1.0 -lharfbuzz -lfontconfig -lfreetype -ldl -lsecret-1 -lgio-2.0 -lgobject-2.0 -lglib-2.0 -lm

all: checkLibs mkbuild $(b)/configurator cpylib

# Make sure libraries are built
.PHONY: checkLibs
checkLibs:
	@test -d $(cvl) || { echo "OpenCV library is missing ($(cvl))!"; exit 1; }
	@test -d $(wxl) || { echo "wxWidgets library is missing ($(wxl))!"; exit 1; }
	@test -d include/opencv2 || { echo "OpenCV include files are missing (include/opencv2)!"; exit 1; }
	@test -d include/wx || { echo "wxWidgets include files are missing (include/wx)!"; exit 1; }

.PHONY: mkbuild
mkbuild:
	@if [ ! -d $(b) ]; then mkdir $(b); fi
	@if [ ! -d $(o) ]; then mkdir $(o); fi

.PHONY: cpylib
cpylib:
	cp $(wildcard $(libs)/libusb/libusb*.so*) "$(b)/"

# Configurator application
$(b)/configurator: $(o)/configurator.obj $(o)/comm.obj $(o)/config.obj $(o)/control.obj $(o)/testing.obj $(o)/calibration.obj $(o)/tracking.obj $(o)/ui.obj $(o)/visualization.obj $(o)/mesh.obj $(o)/glew.obj
	g++ -o $@ $^ $(wxlibs) $(cvlibs) $(usblibs) $(syslibs) $(ldebug)

# Add include dependencies (could be done automatically on linux, but not so on windows, so manual for all)
$(o)/calibration.obj: $(i)/calibration.hpp $(i)/wxbase.hpp $(i)/eigenutil.hpp
$(o)/comm.obj: $(i)/comm.hpp $(i)/util.hpp $(i)/wxbase.hpp
$(o)/config.obj: $(i)/config.hpp $(i)/control.hpp $(i)/calibration.hpp $(i)/tracking.hpp $(i)/eigenutil.hpp
$(o)/configurator.obj: $(i)/configurator.hpp $(i)/config.hpp $(i)/control.hpp $(i)/util.hpp $(i)/testing.hpp $(i)/calibration.hpp $(i)/tracking.hpp $(i)/comm.hpp $(i)/wxbase.hpp $(i)/eigenutil.hpp
$(o)/control.obj: $(i)/control.hpp $(i)/config.hpp $(i)/calibration.hpp $(i)/tracking.hpp $(i)/wxbase.hpp $(i)/eigenutil.hpp
$(o)/mesh.obj: $(i)/mesh.hpp
$(o)/testing.obj: $(i)/testing.hpp $(i)/tracking.hpp $(i)/wxbase.hpp $(i)/eigenutil.hpp
$(o)/tracking.obj: $(i)/tracking.hpp $(i)/wxbase.hpp $(i)/eigenutil.hpp
$(o)/ui.obj: $(i)/ui.hpp $(i)/configurator.hpp $(i)/calibration.hpp $(i)/tracking.hpp $(i)/visualization.hpp $(i)/eigenutil.hpp
$(o)/visualization.obj: $(i)/visualization.hpp $(i)/calibration.hpp $(i)/tracking.hpp $(i)/mesh.hpp $(i)/eigenutil.hpp