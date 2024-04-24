TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = pylonApp
PROD_IOC_WIN32 += $(PROD_NAME)
ifeq ($(WITH_PYLON), YES)
PROD_IOC_Linux += $(PROD_NAME)
USR_LDFLAGS_Linux += -L $(PYLON_LIB) -Wl,-rpath,$(PYLON_LIB)
PROD_SYS_LIBS_Linux  += GenApi_gcc_v3_1_Basler_pylon GCBase_gcc_v3_1_Basler_pylon pylonbase pylonutility 
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

