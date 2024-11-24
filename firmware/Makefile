.PHONY: all clean

DIR_SRC := src
DIR_BUILD := build
DIR_TOOLS := tools

APP_NAME := battery_checker

MCU := attiny13a

OBJ_LIST := $(DIR_BUILD)/${APP_NAME}.o
ELF := $(DIR_BUILD)/$(APP_NAME).elf
HEX := $(DIR_BUILD)/$(APP_NAME).hex

CXX := avr-g++
OBJCOPY := avr-objcopy
AVR_SIZE := avr-size

CXXFLAGS := -g -Os -mmcu=$(MCU)
LDFLAGS := -g -Os -mmcu=$(MCU)

all: $(HEX)

$(HEX): $(ELF)
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

$(ELF): $(OBJ_LIST)
	$(CXX) $(LDFLAGS) -o $@ $(OBJ_LIST)
	$(AVR_SIZE) -C --mcu $(MCU) $(ELF) | grep bytes

$(DIR_BUILD)/%.o: $(DIR_SRC)/%.cpp
	mkdir -p $(DIR_BUILD)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

include $(DIR_TOOLS)/atprogram-win/tools.mk

clean:
	rm -rf $(DIR_BUILD)


