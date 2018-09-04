# Installation directory.
VER = 16.0
INSTDIR = $(HOME)/houdini$(VER)/dso/mantra
HFS = /opt/hfs16

# List of C++ source files to build.
# registers the operators and handles the DSO-specifics.
SOURCES = \
    ./src$(VER)/RAY_Deformer.cpp \
    ./src$(VER)/RAY_DeformInstance.cpp

# Use the highest optimization level.
OPTIMIZER = -O3

# Set the plugin library name.
DSONAME = $(INSTDIR)/VRAY_DemoBox.so

# Include the GNU Makefile.
include $(HFS)/toolkit/makefiles/Makefile.gnu
