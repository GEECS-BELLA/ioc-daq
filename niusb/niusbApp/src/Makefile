TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += NiUsbProtocol.h NiUsbTransport.h NiUsbFirmware.h NiUsb621x.h NiUsbFake.h NiUsbLibusb.h
INC += NiUsb6008.h NiUsb6008Fake.h
LIBRARY_IOC = niusb
LIB_SRCS += NiUsbProtocol.cpp NiUsbFirmware.cpp NiUsb621x.cpp NiUsbFake.cpp NiUsbLibusb.cpp
LIB_SRCS += NiUsb6008.cpp NiUsb6008Fake.cpp
LIB_SRCS += drvNiUsb621x.cpp drvNiUsb6008.cpp
DBD += niusbSupport.dbd
USR_CXXFLAGS += -std=c++17
LIB_SYS_LIBS += usb-1.0
niusb_LIBS += asyn
niusb_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
