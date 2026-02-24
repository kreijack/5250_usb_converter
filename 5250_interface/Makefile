
# configure here
ARDUINOCLI=~/local/arduino-cli-1.2.0/arduino-cli_1.2.0_Linux_64bit/arduino-cli
BOARD=teensy:avr:teensy40
PORT=usb3/3-1/3-1.4/3-1.4.4

DIRNAME=$(shell basename $(CURDIR))
CXXFLAGS=-Wall -pedantic

# Arduino convention
SRC=$(DIRNAME).ino

BUILDDIR=build
HEXFILE=$(BUILDDIR)/$(SRC).hex


all: $(HEXFILE)

clean:
	rm -rf ./$(BUILDDIR)/

$(HEXFILE): $(SRC)
	$(ARDUINOCLI) compile -b $(BOARD) -v -p $(PORT) \
		--build-property compiler.cpp.extra_flags="$(CXXFLAGS)" \
		-e --build-path $(BUILDDIR)

upload: $(HEXFILE)
	$(ARDUINOCLI) upload -b $(BOARD) -v -p $(PORT) -i $(HEXFILE)



