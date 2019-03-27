EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:zeabus
LIBS:PCIe_pinout-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L +12V #PWR?
U 1 1 5A63A90A
P 1850 2000
F 0 "#PWR?" H 1850 1850 50  0001 C CNN
F 1 "+12V" H 1850 2140 50  0000 C CNN
F 2 "" H 1850 2000 50  0001 C CNN
F 3 "" H 1850 2000 50  0001 C CNN
	1    1850 2000
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 5A63A966
P 3800 2100
F 0 "#PWR?" H 3800 1950 50  0001 C CNN
F 1 "+12V" H 3800 2240 50  0000 C CNN
F 2 "" H 3800 2100 50  0001 C CNN
F 3 "" H 3800 2100 50  0001 C CNN
	1    3800 2100
	1    0    0    -1  
$EndComp
$Comp
L Conn-PCIE-x16 CON1
U 1 1 5A63A836
P 2450 2000
F 0 "CON1" H 2600 2150 60  0000 C CNN
F 1 "PCIe_pinout_standard" H 2800 2050 60  0000 C CNN
F 2 "" H 2450 2000 60  0001 C CNN
F 3 "" H 2450 2000 60  0001 C CNN
	1    2450 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2200 3800 2200
$Comp
L +5V #PWR?
U 1 1 5A63AC89
P 3900 2800
F 0 "#PWR?" H 3900 2650 50  0001 C CNN
F 1 "+5V" H 3900 2940 50  0000 C CNN
F 2 "" H 3900 2800 50  0001 C CNN
F 3 "" H 3900 2800 50  0001 C CNN
	1    3900 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 3000 3500 3000
Wire Wire Line
	3500 3000 3500 2900
Wire Wire Line
	3400 2300 3500 2300
Wire Wire Line
	3500 2300 3500 2200
Connection ~ 3500 2200
Wire Wire Line
	2250 2200 2150 2200
Wire Wire Line
	2150 2100 2150 2300
Connection ~ 2150 2100
Connection ~ 2150 2200
Wire Wire Line
	2150 2300 2250 2300
Wire Wire Line
	1850 2000 1850 2100
Wire Wire Line
	1850 2100 2250 2100
$Comp
L +5V #PWR?
U 1 1 5A63AF13
P 1750 2700
F 0 "#PWR?" H 1750 2550 50  0001 C CNN
F 1 "+5V" H 1750 2840 50  0000 C CNN
F 2 "" H 1750 2700 50  0001 C CNN
F 3 "" H 1750 2700 50  0001 C CNN
	1    1750 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2700 1750 2800
Wire Wire Line
	1750 2800 2250 2800
$Comp
L GND #PWR?
U 1 1 5A63B051
P 2050 4000
F 0 "#PWR?" H 2050 3750 50  0001 C CNN
F 1 "GND" H 2050 3850 50  0000 C CNN
F 2 "" H 2050 4000 50  0001 C CNN
F 3 "" H 2050 4000 50  0001 C CNN
	1    2050 4000
	1    0    0    -1  
$EndComp
Text GLabel 1650 3400 0    60   BiDi ~ 0
data+
Text GLabel 1650 3700 0    60   BiDi ~ 0
data-
$Comp
L Conn-PCIE-x16 CON2
U 1 1 5A63B4F1
P 5600 2000
F 0 "CON2" H 5750 2150 60  0000 C CNN
F 1 "PCIe_pinout_power_dist_addtional_pin" H 5950 2050 60  0000 C CNN
F 2 "" H 5600 2000 60  0001 C CNN
F 3 "" H 5600 2000 60  0001 C CNN
	1    5600 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2300 6950 2300
Wire Wire Line
	6950 2300 6950 2200
Wire Wire Line
	3800 2200 3800 2100
Wire Wire Line
	3900 2900 3900 2800
Wire Wire Line
	3400 2900 3900 2900
Connection ~ 3500 2900
Wire Wire Line
	2050 2400 2050 4000
Wire Wire Line
	2050 3900 2250 3900
Wire Wire Line
	2250 2400 2050 2400
Connection ~ 2050 3900
Wire Wire Line
	2250 2700 2050 2700
Connection ~ 2050 2700
Wire Wire Line
	2250 3400 2050 3400
Connection ~ 2050 3400
Wire Wire Line
	2250 3700 2050 3700
Connection ~ 2050 3700
$Comp
L GND #PWR?
U 1 1 5A63BD5E
P 3600 4000
F 0 "#PWR?" H 3600 3750 50  0001 C CNN
F 1 "GND" H 3600 3850 50  0000 C CNN
F 2 "" H 3600 4000 50  0001 C CNN
F 3 "" H 3600 4000 50  0001 C CNN
	1    3600 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 3900 3600 3900
Wire Wire Line
	3600 2400 3600 4000
Wire Wire Line
	3600 2400 3400 2400
Connection ~ 3600 3900
Wire Wire Line
	3400 3300 3600 3300
Connection ~ 3600 3300
Wire Wire Line
	3400 3600 3600 3600
Connection ~ 3600 3600
Wire Wire Line
	1650 3400 1700 3400
Wire Wire Line
	1700 3400 1700 3500
Wire Wire Line
	1700 3500 2250 3500
Wire Wire Line
	1650 3700 1700 3700
Wire Wire Line
	1700 3700 1700 3600
Wire Wire Line
	1700 3600 2250 3600
Text GLabel 7000 2200 2    60   Output ~ 0
+VBatt
Wire Wire Line
	6950 2200 7000 2200
Text GLabel 5150 2800 0    60   Output ~ 0
GND_VBatt
Wire Wire Line
	5150 2800 5200 2800
Wire Wire Line
	5200 2800 5200 2700
Wire Wire Line
	5200 2700 5400 2700
Text GLabel 5000 3000 0    60   Input ~ 0
+5V_power_controller
Wire Wire Line
	5000 3000 5400 3000
Text GLabel 5150 3800 0    60   Input ~ 0
GND_power_controller
Wire Wire Line
	5400 3700 5200 3700
Wire Wire Line
	5200 3700 5200 3800
Wire Wire Line
	5200 3800 5150 3800
$Comp
L Conn-PCIE-x16 CON3
U 1 1 5A63D475
P 8300 2000
F 0 "CON3" H 8450 2150 60  0000 C CNN
F 1 "PCIe_pinout_peripheral_bridge_addtional_pin" H 8650 2050 60  0000 C CNN
F 2 "" H 8300 2000 60  0001 C CNN
F 3 "" H 8300 2000 60  0001 C CNN
	1    8300 2000
	1    0    0    -1  
$EndComp
Text GLabel 7850 2500 0    60   Input ~ 0
pressure_sensor_analog
Wire Wire Line
	7850 2500 7900 2500
Wire Wire Line
	7900 2500 7900 2600
Wire Wire Line
	7900 2600 8100 2600
Text GLabel 9500 2750 2    60   Output ~ 0
TxD_RS232
Text GLabel 9500 2650 2    60   Input ~ 0
RxD_RS232
Wire Wire Line
	9250 2600 9450 2600
Text GLabel 9500 2500 2    60   Output ~ 0
GND_RS232
Wire Wire Line
	9450 2600 9450 2650
Wire Wire Line
	9500 2750 9450 2750
Wire Wire Line
	9450 2750 9450 2700
Wire Wire Line
	9450 2700 9250 2700
Wire Wire Line
	9450 2650 9500 2650
Wire Wire Line
	9500 2500 9450 2500
Wire Wire Line
	9450 2500 9450 2400
Wire Wire Line
	9450 2400 9250 2400
Text GLabel 9450 3300 2    60   UnSpc ~ 0
solenoid_valve1
Wire Wire Line
	9250 3300 9450 3300
Wire Wire Line
	9450 3900 9250 3900
Wire Wire Line
	9250 3800 9450 3800
Wire Wire Line
	9250 3700 9450 3700
Wire Wire Line
	9250 3600 9450 3600
Wire Wire Line
	9250 3500 9450 3500
Wire Wire Line
	9250 3400 9450 3400
Text Label 2650 5750 0    60   ~ 0
Standart
Text Label 5300 5750 0    60   ~ 0
Power_distributor_addtional_pin
Text Label 8000 5750 0    60   ~ 0
Peripheral_bridge_addtional_pin
Text GLabel 7850 3100 0    60   UnSpc ~ 0
DSP_reset+
Wire Wire Line
	7850 3100 8100 3100
Text GLabel 9450 3100 2    60   UnSpc ~ 0
DSP_reset-
Wire Wire Line
	9450 3100 9250 3100
Text GLabel 7850 3300 0    60   Input ~ 0
planner_sw
Wire Wire Line
	8100 3300 7850 3300
Text GLabel 9450 3400 2    60   UnSpc ~ 0
solenoid_valve2
Text GLabel 9450 3500 2    60   UnSpc ~ 0
solenoid_valve3
Text GLabel 9450 3600 2    60   UnSpc ~ 0
solenoid_valve4
Text GLabel 9450 3700 2    60   UnSpc ~ 0
solenoid_valve5
Text GLabel 9450 3800 2    60   UnSpc ~ 0
solenoid_valve6
Text GLabel 9450 3900 2    60   UnSpc ~ 0
solenoid_valve7
Text GLabel 6750 3400 2    60   UnSpc ~ 0
KILL_SW+
Text GLabel 6750 3700 2    60   UnSpc ~ 0
SYS_SW+
Text GLabel 6750 3500 2    60   UnSpc ~ 0
KILL_SW-
Wire Wire Line
	6550 3400 6750 3400
Wire Wire Line
	6750 3500 6550 3500
Text GLabel 6750 3800 2    60   UnSpc ~ 0
SYS_SW+
Wire Wire Line
	6750 3700 6550 3700
Wire Wire Line
	6550 3800 6750 3800
$EndSCHEMATC
