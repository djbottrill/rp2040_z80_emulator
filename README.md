Z80 Emulator for Raspberry Pi Pico and other RP2040 based MCU boards.


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

