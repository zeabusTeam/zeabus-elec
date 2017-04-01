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
Sheet 5 5
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
L UWE-12/6-Q12P U?
U 1 1 58DFE00C
P 5600 3400
F 0 "U?" H 5300 3800 60  0000 C CNN
F 1 "UWE-12/6-Q12P" H 5600 3000 60  0000 C CNN
F 2 "" H 5650 3100 60  0000 C CNN
F 3 "" H 5650 3100 60  0000 C CNN
	1    5600 3400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DFE329
P 4100 3750
F 0 "#PWR?" H 4100 3500 50  0001 C CNN
F 1 "GND" H 4100 3600 50  0000 C CNN
F 2 "" H 4100 3750 50  0001 C CNN
F 3 "" H 4100 3750 50  0001 C CNN
	1    4100 3750
	1    0    0    -1  
$EndComp
$Comp
L Jumper JP?
U 1 1 58DFE33F
P 4600 3550
F 0 "JP?" H 4600 3700 50  0000 C CNN
F 1 "Jumper" H 4600 3470 50  0000 C CNN
F 2 "" H 4600 3550 50  0001 C CNN
F 3 "" H 4600 3550 50  0001 C CNN
	1    4600 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3400 5050 3550
Wire Wire Line
	5050 3550 4900 3550
Wire Wire Line
	5050 3250 4100 3250
Wire Wire Line
	4100 3250 4100 3750
Wire Wire Line
	4300 3550 4100 3550
Connection ~ 4100 3550
$Comp
L LED_Small D?
U 1 1 58DFE475
P 6700 3750
F 0 "D?" H 6650 3875 50  0000 L CNN
F 1 "LED_Small" H 6525 3650 50  0000 L CNN
F 2 "" V 6700 3750 50  0001 C CNN
F 3 "" V 6700 3750 50  0001 C CNN
	1    6700 3750
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 58DFE4C6
P 6700 3350
F 0 "R?" V 6780 3350 50  0000 C CNN
F 1 "R" V 6700 3350 50  0000 C CNN
F 2 "" V 6630 3350 50  0001 C CNN
F 3 "" H 6700 3350 50  0001 C CNN
	1    6700 3350
	1    0    0    -1  
$EndComp
$Comp
L XT60 P?
U 1 1 58DFE4FF
P 7350 3550
F 0 "P?" H 7350 3700 50  0000 C CNN
F 1 "XT60" H 7350 3400 50  0000 C CNN
F 2 "" H 7350 3550 60  0000 C CNN
F 3 "" H 7350 3550 60  0000 C CNN
	1    7350 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 3100 7150 3100
Wire Wire Line
	6700 3100 6700 3200
Wire Wire Line
	6700 3500 6700 3650
Wire Wire Line
	6150 3700 6250 3700
Wire Wire Line
	6250 3700 6250 4000
Wire Wire Line
	6250 4000 7150 4000
Wire Wire Line
	6700 4000 6700 3850
Wire Wire Line
	7150 3100 7150 3500
Connection ~ 6700 3100
Wire Wire Line
	7150 4000 7150 3600
Connection ~ 6700 4000
Text HLabel 3250 3100 0    60   Input ~ 0
+Power
Wire Wire Line
	3250 3100 5050 3100
$EndSCHEMATC
