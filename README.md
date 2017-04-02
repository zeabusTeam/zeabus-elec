# Zeabus-Elec-2017-Code
<<<<<<< HEAD
Library for Power Distributer and Peripheral Bridge

## The base FTDI library
The base library consists of 2 files, _libftd2xx.a_ and _ftd2xx.h_.

### The header file, _ftd2xx.h_, was modified to conform the standard integer defined  in `stdint.h`. Therefore, the file `WinTypes.h`, included in the package, is unnecessary.

The library requires "pthread" and "dl" libraries provided by the system.
Therefore, to build the program we need _-lpthread -ldl_ option. 
For example `gcc main.c -o runing libftd2xx -lpthread -ldl`.

## Zeabus Library
Zebus library was built over the base FTDI library. It provides high-level functions to control various components via FTxxx chips.

The components under control of FTxx are:

* _FT232H_ The chip must be pre-programmed its serial number as "PowerDist".
  * 8-Channel galvanic-isolated power switches assigned as:
    * 6 channels for thrusters, DVL, and Sonar (including spared channels) with directed connection to batteries
    * 1 channel for head lamps with 12V supply
    * 1 channel for DSP board with 5V supply
* _FT4232H_ This chip consists of 4 independent modules Their serial numbers must be pre-programmed as "PerpheralBridge-A", "PerpheralBridge-B", "PerpheralBridge-C", and "PerpheralBridge-D".
  * 8-Channel galvanic-isolated switches for pneumatic valves.
  * 2-Channel galvanic-isolated RS232 for communication with DSP and DVL boards

The library consists of 2 files, _zblib.h_ and _zblib.c_. Users should consider only information in the .h file.

Normally, Zeabus team use only this Zeabus library and let the library manipulate base FTDI function calls.
=======
Library for Power Distributer and Peripheral Bridge\
>>>>>>> 27a5e12838015e11ddaf8a76199aba14b08e9631
