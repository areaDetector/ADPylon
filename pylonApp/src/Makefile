TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += ADPylon

# External Python Linux SDK
ifeq ($(WITH_PYLON), YES)
  LIBRARY_IOC_Linux += ADPylon

  USR_CPPFLAGS_Linux += -I $(PYLON_INCLUDE)
  USR_LDFLAGS_Linux += -L $(PYLON_LIB) -Wl,-rpath,$(PYLON_LIB)
  LIB_SYS_LIBS_Linux  += pylonbase pylonutility
endif

# Pylon Windows SDK
USR_INCLUDES_WIN32 += -I ../../../pylonSupport/include

LIB_LIBS_WIN32  += GenApi_MD_VC142_v3_5_Basler_pylon_v1
LIB_LIBS_WIN32  += GCBase_MD_VC142_v3_5_Basler_pylon_v1
LIB_LIBS_WIN32  += PylonUtility_v11
LIB_LIBS_WIN32  += PylonBase_v11

LIB_SRCS += PylonFeature.cpp
LIB_SRCS += ADPylon.cpp

LIB_LIBS += ADGenICam

DBD += ADPylonSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

