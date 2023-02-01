TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += ADPylon

LIB_SRCS += PylonFeature.cpp
LIB_SRCS += ADPylon.cpp

LIB_LIBS += ADGenICam

# Pylon SDK
USR_INCLUDES += -I ../../../pylonSupport/include

LIB_LIBS_WIN32  += GenApi_MD_VC141_v3_1_Basler_pylon
LIB_LIBS_WIN32  += GCBase_MD_VC141_v3_1_Basler_pylon
LIB_LIBS_WIN32  += PylonUtility_v7_2
LIB_LIBS_WIN32  += PylonBase_v7_2

DBD += ADPylonSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

