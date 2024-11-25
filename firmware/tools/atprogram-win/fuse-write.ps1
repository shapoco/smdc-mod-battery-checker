&"C:\Program Files (x86)\Atmel\Studio\7.0\atbackend\atprogram.exe" `
    -t avrispmk2 `
    -i isp `
    -d attiny13a `
    write `
    --fuses `
    --values 69FD `
    --verify

# Fuse High Byte [7:0]
#   [7:5]   Reserved                                            0b111
#   [4]     SELFPRGEN       Self Programming Enable             0b1     Disabled
#   [3]     DWEN            debugWire Enable                    0b1     Disabled
#   [2:1]   BODLEVEL[1:0]   Brown-out Detector trigger level    0b10    1.8V
#   [0]     RSTDISBL        External Reset disable              0b1     Enabled

# Fuse Low Byte [7:0]
#   [7]     SPIEN           Enable Serial Programming           0b0     Enabled
#   [6]     EESAVE          Preserve EEPROM memory              0b1     Disabled
#   [5]     WDTON           Watchdog Timer always on            0b1     Disabled
#   [4]     CKDIV8          Divide clock by 8                   0b0     Enabled
#   [3:2]   SUT[1:0]        Select start-up time                0b10    Slow
#   [1:0]   CKSEL[1:0]      Select clock source                 0b01    4.8 MHz

# BODLEVEL [1:0]
#   00 4.3V
#   01 2.7V
#   10 1.8V
#   11 Disabled

# SUT [1:0]
#   00 6CK + 14CK
#   01 6CK + 14CK + 4ms
#   10 6CK + 14CK + 64ms
#   11 Reserved

# CKSEL [1:0]
#   00 External clock
#   01 Calibrated internal 4.8 MHz oscillator
#   10 Calibrated internal 9.6 MHz oscillator
#   11 Internal 128kHz oscillator
