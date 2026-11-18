# Native builds of the niusb module's plain-C++ parts (no EPICS, no libusb),
# driven by tests/test_cpp.py: the capture replay and the fake.
SRC = ../../niusb/niusbApp/src
CXX ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -I$(SRC) -I.
OUT ?= build
CORE = $(SRC)/NiUsbProtocol.cpp $(SRC)/NiUsbFirmware.cpp $(SRC)/NiUsb621x.cpp $(SRC)/NiUsbFake.cpp

all: $(OUT)/niusb_replay $(OUT)/niusb_fake $(OUT)/sha_check

$(OUT):
	mkdir -p $(OUT)

$(OUT)/niusb_replay: niusb_replay.cpp $(CORE) $(SRC)/*.h | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ niusb_replay.cpp $(CORE)

$(OUT)/niusb_fake: niusb_fake.cpp $(CORE) $(SRC)/*.h | $(OUT)
	$(CXX) $(CXXFLAGS) -pthread -o $@ niusb_fake.cpp $(CORE)

$(OUT)/sha_check: sha_check.cpp $(SRC)/NiUsbFirmware.cpp $(SRC)/NiUsbProtocol.cpp $(SRC)/*.h | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ sha_check.cpp $(SRC)/NiUsbFirmware.cpp $(SRC)/NiUsbProtocol.cpp

clean:
	rm -rf $(OUT)
