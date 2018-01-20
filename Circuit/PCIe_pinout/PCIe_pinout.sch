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
P 1900 2000
F 0 "#PWR?" H 1900 1850 50  0001 C CNN
F 1 "+12V" H 1900 2140 50  0000 C CNN
F 2 "" H 1900 2000 50  0001 C CNN
F 3 "" H 1900 2000 50  0001 C CNN
	1    1900 2000
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 5A63A966
P 3850 2100
F 0 "#PWR?" H 3850 1950 50  0001 C CNN
F 1 "+12V" H 3850 2240 50  0000 C CNN
F 2 "" H 3850 2100 50  0001 C CNN
F 3 "" H 3850 2100 50  0001 C CNN
	1    3850 2100
	1    0    0    -1  
$EndComp
$Comp
L Conn-PCIE-x16 CON?
U 1 1 5A63A836
P 2500 2000
F 0 "CON?" H 2650 2150 60  0000 C CNN
F 1 "PCIe_pinout_standard" H 2850 2050 60  0000 C CNN
F 2 "" H 2500 2000 60  0001 C CNN
F 3 "" H 2500 2000 60  0001 C CNN
	1    2500 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2200 3850 2200
$Comp
L +5V #PWR?
U 1 1 5A63AC89
P 3950 2800
F 0 "#PWR?" H 3950 2650 50  0001 C CNN
F 1 "+5V" H 3950 2940 50  0000 C CNN
F 2 "" H 3950 2800 50  0001 C CNN
F 3 "" H 3950 2800 50  0001 C CNN
	1    3950 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3000 3550 3000
Wire Wire Line
	3550 3000 3550 2900
Wire Wire Line
	3450 2300 3550 2300
Wire Wire Line
	3550 2300 3550 2200
Connection ~ 3550 2200
Wire Wire Line
	2300 2200 2200 2200
Wire Wire Line
	2200 2100 2200 2300
Connection ~ 2200 2100
Connection ~ 2200 2200
Wire Wire Line
	2200 2300 2300 2300
Wire Wire Line
	1900 2000 1900 2100
Wire Wire Line
	1900 2100 2300 2100
$Comp
L +5V #PWR?
U 1 1 5A63AF13
P 1800 2700
F 0 "#PWR?" H 1800 2550 50  0001 C CNN
F 1 "+5V" H 1800 2840 50  0000 C CNN
F 2 "" H 1800 2700 50  0001 C CNN
F 3 "" H 1800 2700 50  0001 C CNN
	1    1800 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2700 1800 2800
Wire Wire Line
	1800 2800 2300 2800
$Comp
L GND #PWR?
U 1 1 5A63B051
P 2100 4000
F 0 "#PWR?" H 2100 3750 50  0001 C CNN
F 1 "GND" H 2100 3850 50  0000 C CNN
F 2 "" H 2100 4000 50  0001 C CNN
F 3 "" H 2100 4000 50  0001 C CNN
	1    2100 4000
	1    0    0    -1  
$EndComp
Text GLabel 1700 3400 0    60   Input ~ 0
Data+
Text GLabel 1700 3700 0    60   Input ~ 0
Data-
$Comp
L Conn-PCIE-x16 CON?
U 1 1 5A63B4F1
P 5650 2000
F 0 "CON?" H 5800 2150 60  0000 C CNN
F 1 "PCIe_pinout_power_dist_addtional_pin" H 6000 2050 60  0000 C CNN
F 2 "" H 5650 2000 60  0001 C CNN
F 3 "" H 5650 2000 60  0001 C CNN
	1    5650 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2300 7000 2300
Wire Wire Line
	7000 2300 7000 2200
Wire Wire Line
	3850 2200 3850 2100
Wire Wire Line
	3950 2900 3950 2800
Wire Wire Line
	3450 2900 3950 2900
Connection ~ 3550 2900
Wire Wire Line
	2100 2400 2100 4000
Wire Wire Line
	2100 3900 2300 3900
Wire Wire Line
	2300 2400 2100 2400
Connection ~ 2100 3900
Wire Wire Line
	2300 2700 2100 2700
Connection ~ 2100 2700
Wire Wire Line
	2300 3400 2100 3400
Connection ~ 2100 3400
Wire Wire Line
	2300 3700 2100 3700
Connection ~ 2100 3700
$Comp
L GND #PWR?
U 1 1 5A63BD5E
P 3650 4000
F 0 "#PWR?" H 3650 3750 50  0001 C CNN
F 1 "GND" H 3650 3850 50  0000 C CNN
F 2 "" H 3650 4000 50  0001 C CNN
F 3 "" H 3650 4000 50  0001 C CNN
	1    3650 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3900 3650 3900
Wire Wire Line
	3650 2400 3650 4000
Wire Wire Line
	3650 2400 3450 2400
Connection ~ 3650 3900
Wire Wire Line
	3450 3300 3650 3300
Connection ~ 3650 3300
Wire Wire Line
	3450 3600 3650 3600
Connection ~ 3650 3600
Wire Wire Line
	1700 3400 1750 3400
Wire Wire Line
	1750 3400 1750 3500
Wire Wire Line
	1750 3500 2300 3500
Wire Wire Line
	1700 3700 1750 3700
Wire Wire Line
	1750 3700 1750 3600
Wire Wire Line
	1750 3600 2300 3600
Text GLabel 7050 2200 2    60   Input ~ 0
+VBatt
Wire Wire Line
	7000 2200 7050 2200
Text GLabel 5200 2800 0    60   Input ~ 0
GND_VBatt
Wire Wire Line
	5200 2800 5250 2800
Wire Wire Line
	5250 2800 5250 2700
Wire Wire Line
	5250 2700 5450 2700
Text GLabel 5050 3000 0    60   Input ~ 0
+5V_power_controller
Wire Wire Line
	5050 3000 5450 3000
Text GLabel 5200 3800 0    60   Input ~ 0
GND_power_controller
Wire Wire Line
	5450 3700 5250 3700
Wire Wire Line
	5250 3700 5250 3800
Wire Wire Line
	5250 3800 5200 3800
$Comp
L Conn-PCIE-x16 CON?
U 1 1 5A63D475
P 8350 2000
F 0 "CON?" H 8500 2150 60  0000 C CNN
F 1 "PCIe_pinout_peripheral_bridge_addtional_pin" H 8700 2050 60  0000 C CNN
F 2 "" H 8350 2000 60  0001 C CNN
F 3 "" H 8350 2000 60  0001 C CNN
	1    8350 2000
	1    0    0    -1  
$EndComp
Text GLabel 7900 2500 0    60   Input ~ 0
analog_pressure_sensor
Wire Wire Line
	7900 2500 7950 2500
Wire Wire Line
	7950 2500 7950 2600
Wire Wire Line
	7950 2600 8150 2600
Text GLabel 9550 2750 2    60   Input ~ 0
TxD_RS232_DVL
Text GLabel 9550 2650 2    60   Input ~ 0
RxD_RS232_DVL
Wire Wire Line
	9300 2600 9500 2600
Text GLabel 9550 2500 2    60   Input ~ 0
GND_RS232_DVL
Wire Wire Line
	9500 2600 9500 2650
Wire Wire Line
	9550 2750 9500 2750
Wire Wire Line
	9500 2750 9500 2700
Wire Wire Line
	9500 2700 9300 2700
Wire Wire Line
	9500 2650 9550 2650
Wire Wire Line
	9550 2500 9500 2500
Wire Wire Line
	9500 2500 9500 2400
Wire Wire Line
	9500 2400 9300 2400
Text GLabel 9500 3300 2    60   Input ~ 0
solenoid_value_1
Wire Wire Line
	9300 3300 9500 3300
Text GLabel 9500 3400 2    60   Input ~ 0
solenoid_value_2
Text GLabel 9500 3500 2    60   Input ~ 0
solenoid_value_3
Text GLabel 9500 3600 2    60   Input ~ 0
solenoid_value_4
Text GLabel 9500 3700 2    60   Input ~ 0
solenoid_value_5
Text GLabel 9500 3800 2    60   Input ~ 0
solenoid_value_6
Text GLabel 9500 3900 2    60   Input ~ 0
solenoid_value_7
Wire Wire Line
	9500 3900 9300 3900
Wire Wire Line
	9300 3800 9500 3800
Wire Wire Line
	9300 3700 9500 3700
Wire Wire Line
	9300 3600 9500 3600
Wire Wire Line
	9300 3500 9500 3500
Wire Wire Line
	9300 3400 9500 3400
$EndSCHEMATC
