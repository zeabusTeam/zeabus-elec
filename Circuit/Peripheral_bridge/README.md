# Zeabus-Elec-2017-Peripheral_bridge
## Use relative path to ../Kicad-Libraries for specific components
Bridge circuit for several peripherals. It communicates to NUC via a USB (through USB hub). The specifications are:
1. Use 5V power from USB connector
2. Has 2 **isolated** RS-232 ports (use ADM3251E as the tranceiver/receiver) for:
 * DSP board
 * DVL board
3. Has an analog input and a 5V power-supply for a presure sensor.
4. Has 8-channel digital output with **opto-isolator** for solinoid valves.
