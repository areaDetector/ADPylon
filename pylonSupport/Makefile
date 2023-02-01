TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note, the following files were manually copied from the Pylon SDK lib directories on Windows.
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS    += $(wildcard ../os/windows-x64/*.lib)
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

