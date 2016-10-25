# Wireless GH60 satan Keyboard derived Bitcraze Crazyradio dongle 
This project uses GH60 satan PCB, nRF24 wireless module, LiPo(Li-ion) battery and charger module. I am using HHKB but this layout keyboard is hard to buy and expensive. So I start to build my own HHKB with GH60 satan.

Specification
-------
1. Wireless communication
    * nRF24L+ module on keyboard transfer keys to computer
    * nRF24LU1+ usb dongle receive keys from keyboard
2. Modified keyboard enters to standby mode after 1 second if it doesn't have to do anything.
    * The standby current is measured as 490uA with 4.2V battery 
3. Modified circuits of GH60 satan
    * LED is not controlled by SW and disabled LED backlight circuits
    * nRF24L+ connected
        * PB3: MISO
        * PB2: MOSI
        * PB1: SCK
        * PF4: CE
        * PF5: CSN
    * UART DEBUG
        * PF6: TXD
    * Wakeup from standby by all keys
        * PF7: pull-down control of row1~5
        * Connected PF7 thru register 47K to cathod of diodes from row1, 2, 3, 4, 5 (int0, 1, 2, 3 and 6)
        * Pins col3(PE6) and row5(PD5) exchanged for wake up by row5. Row5 is connected to int6.
        * Before entering to suspend, all rows pull-down and all cols output high
    * Battery powered circuit
        * ATMEGA32U4 runs with 3.3V and 8Mhz
        * Added a battery and a charger
        * Added power mux circuit USB and battery power
    * Other
        * Caps Lock led moved from PB2(MOSI) to PB6(LED pwm backlight of all LEDs)

Build
-------
 * clone git clone https://github.com/jangson/crazyradio-firmware.git -b gh60-satan-wireless
 * cd firmware
 * make
 * Refer keyboard part to https://github.com/jangson/tmk_core_custom.git
 
Flash
-------
 * lsusb and check 1915:7777 Nordic Semiconductor ASA
 * cd usbtools
 * ./cradioFlasher.py
 * Click Browse and select firmware/bin/cradio.bin > Click Flash!
 
Updates
-------
#### 2016/10/25
Initial description

# Bitcraze Crazyradio dongle [![Build Status](https://travis-ci.org/bitcraze/crazyradio-firmware.svg)](https://travis-ci.org/bitcraze/crazyradio-firmware)

Source code and tools for the Bitcraze Crazyradio USB dongle.

See [bitcraze wiki](http://wiki.bitcraze.se/projects:crazyradio:index) for more information about
Crazyradio and the USB protocol used.

## Folders content:
- firmware: The firmware source code
- nrfProg:  SPI programmer that uses jtagkey USB adapter
- usbtools: Python scripts to reset and bootload Crazyradio from command line
- lib: Software libraries to use Crazyradio with Python
- fix_bootloader: Script to revive a Crazyradio

## Building the firmware
Requirement:
  - SDCC
  - Binutils (needs objcopy)

On Ubuntu this can be installed with:
```
sudo apt-get install sdcc binutils
```

On Mac (using homebrew):
```
brew install sdcc
brew install binutils
ln -s /usr/local/bin/gobjcopy /usr/local/bin/objcopy
```

To build the firmware you should navigate to the firmware directory.

### Build for Crazyradio
```
make
```
### Build for Crazyradio PA
```
make CRPA=1
```

## Flashing the Firmware

To flash the firmware run the following from the firmware directory:

```
python ../usbtools/launchBootloader.py
python ../usbtools/nrfbootload.py flash bin/cradio.bin
```

After flashing successfully, you need to replug the Crazyradio.
More details are in the [wiki](https://wiki.bitcraze.io/projects:crazyradio:programming).
