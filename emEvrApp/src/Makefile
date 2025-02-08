TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Use typed rset structure (see 3.16.1 release notes)
USR_CPPFLAGS += -DUSE_TYPED_RSET

# Build an IOC support library
LIBRARY_IOC += emEvrSupport

# Compile and add code to the support library
emEvrSupport_SRCS += xxxRecord.c

# Link locally-provided code into the support library,
# rather than directly into the IOC application, that
# causes problems on Windows DLL builds
emEvrSupport_SRCS += devemEvrVersion.c

emEvrSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = emEvrVersion.h
# Macro name
GENVERSIONMACRO = emEvrVERSION

# Build the IOC application
PROD_IOC = emEvr

# emEvr.dbd will be created and installed
DBD += emEvr.dbd

# emEvr.dbd will include these files:
emEvr_DBD += base.dbd

# emEvr_registerRecordDeviceDriver.cpp derives from emEvr.dbd
emEvr_SRCS += emEvr_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
emEvr_SRCS_DEFAULT += emEvrMain.cpp
emEvr_SRCS_vxWorks += -nil-

# Link in the code from our support library
emEvr_LIBS += emEvrSupport

# To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
ifneq ($(SNCSEQ),)
    # Build sncExample into emEvrSupport
    sncExample_SNCFLAGS += +r
    emEvr_DBD += sncExample.dbd
    # A .stt sequence program is *not* pre-processed:
    emEvrSupport_SRCS += sncExample.stt
    emEvrSupport_LIBS += seq pv
    emEvr_LIBS += seq pv

    # Build sncProgram as a standalone program
    PROD_HOST += sncProgram
    sncProgram_SNCFLAGS += +m
    # A .st sequence program *is* pre-processed:
    sncProgram_SRCS += sncProgram.st
    sncProgram_LIBS += seq pv
    sncProgram_LIBS += $(EPICS_BASE_HOST_LIBS)
endif

# Link QSRV (pvAccess Server) if available
ifdef EPICS_QSRV_MAJOR_VERSION
    emEvr_LIBS += qsrv
    emEvr_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
    emEvr_DBD += PVAServerRegister.dbd
    emEvr_DBD += qsrv.dbd
endif

# Finally link IOC to the EPICS Base libraries
emEvr_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

# Explicit dependency needed for generated header file
devemEvrVersion$(DEP): $(COMMON_DIR)/$(GENVERSION)
