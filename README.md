Z80 Emulator for Raspberry Pi Pico and other RP2040 based MCU boards.

Update 28/04/23

CPM SD coomands: sdfiles, sdpath and sdcopy now send serial output to telnet session.
Other minor tweaks.

*************************

Update 24/04/23

This version checks for an SD card on both SPI 0 and SPI 1, plus there are a few minor improvements.

*************************


V2.2 13/04/23

There are a lot of changes in this version, if an SD card is found the board will now attempt to boot CP/M 2.2 as per the ESP32 version of the emulator.
If a Raspberry Pi Pico W is detected during compilation as per the ESP32 version WiFi and a Telnet server will be enabled.

I've temporarilly dropped support for LCD status displays pending a more flexible approach and in any case it was merely used to show status information and not console output.


*************************

Update 18/04/2022

The CPU emulator function now runs soley on core 1 with all support functions running on core 0. The single Step / breakpoint capability has been streamlined and serial buffering has been re-instaed to improve reliability when pasting basic programs through the serial terminal.

To enable Single Step and or breakpoint capability reset the RP2040 while pressing button A,

Support has beed added for the Pimoroni Pico Explorer expansion board which now displays a representation of the Program Counter, Instructon byte and status of the two 8 bit virtual I/O ports on the 240x240 pixel LCD display.

Note the RP2040 waits for the serial port to connect before booting the emulator.

*************************

Update 24/9/21

Removed the serial buffering task running on the second core as it was not helping performance, this should now enable the code to run on virtually any Arduino compatible core providing there is enough flash program storage and RAM. 

*************************

This is based on my ESP32 Z80 emulator https://github.com/djbottrill/ESP32-Z80-Emulator but simplified to remove the SD card and simply boot into Nascom Basic rather than running CP/M
The ESP32 version could be easilly adapted to run on the RP2040.

The BIOS provides basic emulation of an 8250 UART, GPIO's 0 - 7 and 8 - 15 are mapped to two virtual PIO ports on the Z80 at adresses 0x00 and 0x01 with direction 
control registers at 0x02 and 0x03 respectively. 

The disk command register at port 0x20 is copied from the ESP32 version but only command 8 (reboot) is honoured.

