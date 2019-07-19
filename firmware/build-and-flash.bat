echo off

set CHIP=attiny85

set BIN_FILE=speed_corrector.bin
set HEX_FILE=speed_corrector.hex
del %BIN_FILE%
del %HEX_FILE%

"avr-g++.exe" -Wall -g -Os -mmcu=%CHIP% -o %BIN_FILE% speed_corrector.cpp
"avr-size.exe" -d %BIN_FILE%
"avr-objcopy" -j .text -j .data -O ihex %BIN_FILE% %HEX_FILE%
"avrdude" -p %CHIP% -c usbasp -U flash:w:%HEX_FILE%:i -F -P usb