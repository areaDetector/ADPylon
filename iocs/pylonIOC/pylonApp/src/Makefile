TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = pylonApp
PROD_IOC_WIN32 += $(PROD_NAME)

ifeq ($(STATIC_BUILD), YES)
  PROD_LIBS_WIN32 += PylonUtility_v7_3
  PROD_LIBS_WIN32 += PylonBase_v7_3
  PROD_LIBS_WIN32 += GenApi_MD_VC141_v3_1_Basler_pylon
  PROD_LIBS_WIN32 += GCBase_MD_VC141_v3_1_Basler_pylon
endif

ifeq ($(WITH_PYLON), YES)
  PROD_IOC_Linux += $(PROD_NAME)
    ifeq ($(STATIC_BUILD), YES)
      PROD_LDFLAGS_Linux += -L $(PYLON_LIB) -Wl,-rpath,$(PYLON_LIB)
      PROD_SYS_LIBS_Linux += pylonutility
      PROD_SYS_LIBS_Linux += pylonbase
      PROD_SYS_LIBS_Linux += GenApi_gcc_v3_1_Basler_pylon
      PROD_SYS_LIBS_Linux += GCBase_gcc_v3_1_Basler_pylon
    endif
endif

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += ADPylonSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS += ADPylon
PROD_LIBS += ADGenICam 

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

