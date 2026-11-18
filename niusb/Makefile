# niusb support module: NI USB-621x multifunction DAQs (USB-6210, USB-6211)
# over libusb, without NI's driver: an asynPortDriver (drvNiUsb621x) on our
# decoding of the USB protocol. NI's firmware is loaded at run time from a
# mounted folder and never stored here (geecs-epics decision 23).
TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
DIRS += niusbApp
niusbApp_DEPEND_DIRS += configure

include $(TOP)/configure/RULES_TOP
