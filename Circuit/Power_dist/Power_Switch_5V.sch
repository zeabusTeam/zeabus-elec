EESchema Schematic File Version 2
LIBS:power
LIBS:device
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
LIBS:Power_Distributor-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 5
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
L TEN_20-2411WIN U?
U 1 1 58DFD91D
P 5450 3700
F 0 "U?" H 5150 4050 60  0000 C CNN
F 1 "TEN_20-2411WIN" H 5500 3300 60  0000 C CNN
F 2 "auv:TEN_20WIN" H 5450 3200 60  0000 C CNN
F 3 "" H 5500 3600 60  0000 C CNN
	1    5450 3700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 58DFD99A
P 6450 3800
F 0 "R?" V 6530 3800 50  0000 C CNN
F 1 "R" V 6450 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6380 3800 50  0001 C CNN
F 3 "" H 6450 3800 50  0001 C CNN
	1    6450 3800
	1    0    0    -1  
$EndComp
$Comp
L LED_Small D?
U 1 1 58DFD9EF
P 6450 4200
F 0 "D?" H 6400 4325 50  0000 L CNN
F 1 "LED_Small" H 6275 4100 50  0000 L CNN
F 2 "LEDs:LED_0603" V 6450 4200 50  0001 C CNN
F 3 "" V 6450 4200 50  0001 C CNN
	1    6450 4200
	0    -1   -1   0   
$EndComp
$Comp
L XT60 P?
U 1 1 58DFDA4A
P 7050 4000
F 0 "P?" H 7050 4150 50  0000 C CNN
F 1 "XT60" H 7050 3850 50  0000 C CNN
F 2 "auv:XT60" H 7150 3750 60  0000 C CNN
F 3 "" H 7050 4000 60  0000 C CNN
	1    7050 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DFDAC1
P 3900 4050
F 0 "#PWR?" H 3900 3800 50  0001 C CNN
F 1 "GND" H 3900 3900 50  0000 C CNN
F 2 "" H 3900 4050 50  0001 C CNN
F 3 "" H 3900 4050 50  0001 C CNN
	1    3900 4050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DFDADD
P 4800 4950
F 0 "#PWR?" H 4800 4700 50  0001 C CNN
F 1 "GND" H 4800 4800 50  0000 C CNN
F 2 "" H 4800 4950 50  0001 C CNN
F 3 "" H 4800 4950 50  0001 C CNN
	1    4800 4950
	1    0    0    -1  
$EndComp
$Comp
L Jumper JP?
U 1 1 58DFDAF9
P 4400 3950
F 0 "JP?" H 4400 4100 50  0000 C CNN
F 1 "Jumper" H 4400 3870 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.00mm" H 4400 3950 50  0001 C CNN
F 3 "" H 4400 3950 50  0001 C CNN
	1    4400 3950
	1    0    0    -1  
$EndComp
$Comp
L BC847B Q?
U 1 1 58DFDB54
P 4700 4550
F 0 "Q?" H 4900 4625 50  0000 L CNN
F 1 "BC847B" H 4900 4550 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 4900 4475 50  0001 L CIN
F 3 "" H 4700 4550 50  0001 L CNN
	1    4700 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3950 3900 3950
Wire Wire Line
	3900 3700 3900 4050
Wire Wire Line
	4900 3700 3900 3700
Connection ~ 3900 3950
Wire Wire Line
	4900 3850 4900 3950
Wire Wire Line
	4900 3950 4700 3950
Wire Wire Line
	4800 3950 4800 4350
Connection ~ 4800 3950
Wire Wire Line
	4800 4750 4800 4950
Wire Wire Line
	6000 3550 6850 3550
Wire Wire Line
	6450 3550 6450 3650
Wire Wire Line
	6450 3950 6450 4100
Wire Wire Line
	6450 4300 6450 4450
Wire Wire Line
	6150 4450 6850 4450
Wire Wire Line
	6150 4450 6150 3700
Wire Wire Line
	6150 3700 6000 3700
Wire Wire Line
	6850 3550 6850 3950
Connection ~ 6450 3550
Wire Wire Line
	6850 4450 6850 4050
Connection ~ 6450 4450
Text HLabel 3350 3550 0    60   Input ~ 0
+Power
Text HLabel 3350 4550 0    60   Input ~ 0
+Switch
Wire Wire Line
	3350 3550 4900 3550
$Comp
L R R?
U 1 1 58E00964
P 4150 4550
F 0 "R?" V 4230 4550 50  0000 C CNN
F 1 "R" V 4150 4550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4080 4550 50  0001 C CNN
F 3 "" H 4150 4550 50  0001 C CNN
	1    4150 4550
	0    1    1    0   
$EndComp
Wire Wire Line
	4500 4550 4300 4550
Wire Wire Line
	4000 4550 3350 4550
$EndSCHEMATC
