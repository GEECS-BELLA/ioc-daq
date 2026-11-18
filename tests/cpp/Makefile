# Native builds of the niusb module's plain-C++ parts (no EPICS, no libusb),
# driven by tests/test_cpp.py: the capture replays and the fakes (621x, 6008).
SRC = ../../niusb/niusbApp/src
CXX ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -I$(SRC) -I.
OUT ?= build
CORE = $(SRC)/NiUsbProtocol.cpp $(SRC)/NiUsbFirmware.cpp $(SRC)/NiUsb621x.cpp $(SRC)/NiUsbFake.cpp
CORE6008 = $(SRC)/NiUsbProtocol.cpp $(SRC)/NiUsb6008.cpp $(SRC)/NiUsb6008Fake.cpp

all: $(OUT)/niusb_replay $(OUT)/niusb_fake $(OUT)/sha_check $(OUT)/niusb6008_replay $(OUT)/niusb6008_fake

$(OUT):
	mkdir -p $(OUT)

$(OUT)/niusb_replay: niusb_replay.cpp $(CORE) $(SRC)/*.h | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ niusb_replay.cpp $(CORE)

$(OUT)/niusb_fake: niusb_fake.cpp $(CORE) $(SRC)/*.h | $(OUT)
	$(CXX) $(CXXFLAGS) -pthread -o $@ niusb_fake.cpp $(CORE)

$(OUT)/sha_check: sha_check.cpp $(SRC)/NiUsbFirmware.cpp $(SRC)/NiUsbProtocol.cpp $(SRC)/*.h | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ sha_check.cpp $(SRC)/NiUsbFirmware.cpp $(SRC)/NiUsbProtocol.cpp

$(OUT)/niusb6008_replay: niusb6008_replay.cpp $(CORE6008) $(SRC)/*.h | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ niusb6008_replay.cpp $(CORE6008)

$(OUT)/niusb6008_fake: niusb6008_fake.cpp $(CORE6008) $(SRC)/*.h | $(OUT)
	$(CXX) $(CXXFLAGS) -pthread -o $@ niusb6008_fake.cpp $(CORE6008)

clean:
	rm -rf $(OUT)
