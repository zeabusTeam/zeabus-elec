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
LIBS:ftdi
LIBS:Power_Distributor-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 13
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
U 1 1 58C99133
P 5550 3300
F 0 "U?" H 5250 3650 60  0000 C CNN
F 1 "TEN_20-2411WIN" H 5600 2900 60  0000 C CNN
F 2 "zeabus:TEN_20WIN" H 5550 2800 60  0000 C CNN
F 3 "" H 5600 3200 60  0000 C CNN
	1    5550 3300
	1    0    0    -1  
$EndComp
$Comp
L LED_Small D?
U 1 1 58C99171
P 6600 3850
F 0 "D?" H 6550 3975 50  0000 L CNN
F 1 "LED_Small" H 6425 3750 50  0000 L CNN
F 2 "LEDs:LED_0603" V 6600 3850 50  0001 C CNN
F 3 "" V 6600 3850 50  0001 C CNN
	1    6600 3850
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 58C991CE
P 6600 3450
F 0 "R?" V 6680 3450 50  0000 C CNN
F 1 "1k" V 6600 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6530 3450 50  0001 C CNN
F 3 "" H 6600 3450 50  0001 C CNN
	1    6600 3450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58C991F7
P 4050 3700
F 0 "#PWR?" H 4050 3450 50  0001 C CNN
F 1 "GND" H 4050 3550 50  0000 C CNN
F 2 "" H 4050 3700 50  0001 C CNN
F 3 "" H 4050 3700 50  0001 C CNN
	1    4050 3700
	1    0    0    -1  
$EndComp
$Comp
L Jumper JP?
U 1 1 58C99211
P 4500 3550
F 0 "JP?" H 4500 3700 50  0000 C CNN
F 1 "Jumper" H 4500 3470 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.00mm" H 4500 3550 50  0001 C CNN
F 3 "" H 4500 3550 50  0001 C CNN
	1    4500 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 3450 5000 3550
Wire Wire Line
	5000 3550 4800 3550
Wire Wire Line
	5000 3300 4050 3300
Wire Wire Line
	4050 3300 4050 3700
Wire Wire Line
	4050 3550 4200 3550
Connection ~ 4050 3550
Wire Wire Line
	6600 3150 6600 3300
Wire Wire Line
	6100 3300 6200 3300
Wire Wire Line
	6200 3300 6200 4050
Wire Wire Line
	6600 4050 6600 3950
Wire Wire Line
	6950 3150 6950 3550
Connection ~ 6600 3150
Wire Wire Line
	6950 4050 6950 3650
Connection ~ 6600 4050
Text HLabel 3800 3150 0    60   Input ~ 0
+Power
Wire Wire Line
	3800 3150 5000 3150
$Comp
L XT60 P?
U 1 1 58C9F835
P 7150 3600
F 0 "P?" H 7150 3750 50  0000 C CNN
F 1 "XT60" H 7150 3450 50  0000 C CNN
F 2 "zeabus:XT60" H 7250 3350 60  0000 C CNN
F 3 "" H 7150 3600 60  0000 C CNN
	1    7150 3600
	1    0    0    -1  
$EndComp
$Comp
L XT60 P?
U 1 1 58DFD635
P 7800 3600
F 0 "P?" H 7800 3750 50  0000 C CNN
F 1 "XT60" H 7800 3450 50  0000 C CNN
F 2 "zeabus:XT60" H 7900 3350 60  0000 C CNN
F 3 "" H 7800 3600 60  0000 C CNN
	1    7800 3600
	1    0    0    -1  
$EndComp
Connection ~ 6950 3150
Connection ~ 6950 4050
$Comp
L XT60 P?
U 1 1 58DFD6BF
P 8450 3600
F 0 "P?" H 8450 3750 50  0000 C CNN
F 1 "XT60" H 8450 3450 50  0000 C CNN
F 2 "zeabus:XT60" H 8550 3350 60  0000 C CNN
F 3 "" H 8450 3600 60  0000 C CNN
	1    8450 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3150 8250 3550
Wire Wire Line
	8250 4050 8250 3650
Wire Wire Line
	7600 3550 7600 3150
Connection ~ 7600 3150
Wire Wire Line
	7600 3650 7600 4050
Connection ~ 7600 4050
Wire Wire Line
	6600 3600 6600 3750
Wire Wire Line
	6100 3150 8250 3150
Wire Wire Line
	6200 4050 8250 4050
$EndSCHEMATC
