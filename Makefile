### Generic Makefile.include for Webots controllers, physics plugins, robot
### window libraries, remote control libraries and other libraries
### to be used with GNU make
###
### Platforms: Windows, Mac OS X, Linux
### Languages: C, C++, Java
###
### Authors: Olivier Michel, Yvan Bourquin, Fabien Rohrer
###          Edmund Ronald, Sergei Poskriakov
###
###-----------------------------------------------------------------------------
###
### This file is meant to be included from the Makefile files located in the
### Webots projects subdirectories. It is possible to set a number of variables
### to customize the build process, i.e., add source files, compilation flags,
### include paths, libraries, etc. These variables should be set in your local
### Makefile just before including this Makefile.include. This Makefile.include
### should never be modified.
###
### Here is a description of the variables you may set in your local Makefile:
###
### ---- C Sources ----
### if your program uses several C source files:
### C_SOURCES = my_plugin.c my_clever_algo.c my_graphics.c
###
### ---- C++ Sources ----
### if your program uses several C++ source files:
### CXX_SOURCES = my_plugin.cc my_clever_algo.cpp my_graphics.c++
###
### ---- Compilation options ----
### if special compilation flags are necessary:
### CFLAGS = -Wno-unused-result
###
### ---- Linked libraries ----
### if your program needs additional libraries:
### INCLUDE = -I"/my_library_path/include"
### LIBRARIES = -L"/path/to/my/library" -lmy_library -lmy_other_library
###
### --- Webots included libraries ---
### In your C++ program, if you want to use the C API, add
### USE_C_API = true
### or if you want to link with the Qt framework embedded in Webots:
### QT = core gui widgets network
###
###-----------------------------------------------------------------------------

### Do not modify: this includes Webots global Makefile.include
CXX_SOURCES = nvwa/debug_new.cpp Position.cpp Message.cpp PlumeModel.cpp Posterior.cpp webots_functions.cpp visilibity.cpp SampleBuffer.cpp controller_STE_multi-robot.cpp

WEBOTS_HOME = /home/wjin/Softwares/webots-R2021b/webots
WK4T_LIB = -lk4_webots

USE_C_API = true

INCLUDE = -I"../../libraries/khepera4" -I"$(WEBOTS_HOME)/include/controller/c/" -I"/usr/include/python3.8" -I"/usr/include/eigen3"
LIBRARIES = -g -L"../../libraries/khepera4"  -L"/usr/lib/python3.8/" -L"/usr/lib/python3.8/config-3.8-x86_64-linux-gnu/" -L"$(WEBOTS_HOME)/lib" -lpython3.8 -lk4_webots -lm -lController -lgsl -lgslcblas  

CFLAGS = -std=c++11 -g -fpermissive

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
