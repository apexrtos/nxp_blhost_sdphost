all: blhost sdphost

CXXFLAGS := -Os -flto -fuse-linker-plugin -DLINUX -DBOOTLOADER_HOST -Isrc -Isrc/include
CFLAGS := -Os -flto -fuse-linker-plugin -DLINUX -DBOOTLOADER_HOST -Isrc -Isrc/blfwk
LDLIBS := -ludev

BLFWK := $(addprefix src/blfwk/src/, Bootloader.o BusPal.o BusPalPeripheral.o \
		Command.o DataSource.o DataTarget.o ELFSourceFile.o \
		GHSSecInfo.o IntelHexSourceFile.o Logging.o SBSourceFile.o \
		SDPCommand.o SDPUsbHidPacketizer.o SRecordSourceFile.o \
		SearchPath.o SerialPacketizer.o SourceFile.o StELFFile.o \
		StExecutableImage.o StIntelHexFile.o StSRecordFile.o \
		UartPeripheral.o UsbHidPacketizer.o UsbHidPeripheral.o \
		Value.o format_string.o hid-linux.o jsoncpp.o options.o \
		serial.o utils.o) \
	src/crc/src/crc16.o

blhost: $(BLFWK) proj/blhost/src/blhost.o
	$(CXX) -o $@ $(CXXFLAGS) $^ $(LOADLIBES) $(LDLIBS)

sdphost: $(BLFWK) sdphost.o
	$(CXX) -o $@ $(CXXFLAGS) $^ $(LOADLIBES) $(LDLIBS)

clean:
	rm -f sdphost blhost $(BLFWK) proj/blhost/src/blhost.o sdphost.o
