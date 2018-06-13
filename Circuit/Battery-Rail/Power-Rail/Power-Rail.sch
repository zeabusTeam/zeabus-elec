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
LIBS:Power-Rail-cache
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
L +BATT #PWR01
U 1 1 594D30CF
P 5350 3250
F 0 "#PWR01" H 5350 3100 50  0001 C CNN
F 1 "+BATT" H 5350 3390 50  0000 C CNN
F 2 "" H 5350 3250 50  0001 C CNN
F 3 "" H 5350 3250 50  0001 C CNN
	1    5350 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3500 6850 3500
Wire Wire Line
	5850 3500 5850 3800
Wire Wire Line
	6350 3500 6350 3800
Connection ~ 5850 3500
Wire Wire Line
	6850 3500 6850 3800
Connection ~ 6350 3500
Wire Wire Line
	5350 3250 5350 3800
Connection ~ 5350 3500
$Comp
L GND #PWR02
U 1 1 594D31C8
P 5350 4450
F 0 "#PWR02" H 5350 4200 50  0001 C CNN
F 1 "GND" H 5350 4300 50  0000 C CNN
F 2 "" H 5350 4450 50  0001 C CNN
F 3 "" H 5350 4450 50  0001 C CNN
	1    5350 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3900 5350 4450
Wire Wire Line
	5350 4200 6850 4200
Wire Wire Line
	5850 4200 5850 3900
Connection ~ 5350 4200
Wire Wire Line
	6350 4200 6350 3900
Connection ~ 5850 4200
Wire Wire Line
	6850 4200 6850 3900
Connection ~ 6350 4200
$Comp
L XT60 P1
U 1 1 5A75305B
P 5550 3850
F 0 "P1" H 5550 4000 50  0000 C CNN
F 1 "XT60" H 5550 3700 50  0000 C CNN
F 2 "zeabus:XT60" H 5550 3850 60  0001 C CNN
F 3 "" H 5550 3850 60  0000 C CNN
	1    5550 3850
	1    0    0    -1  
$EndComp
$Comp
L XT60 P2
U 1 1 5A7531B6
P 6050 3850
F 0 "P2" H 6050 4000 50  0000 C CNN
F 1 "XT60" H 6050 3700 50  0000 C CNN
F 2 "zeabus:XT60" H 6050 3850 60  0001 C CNN
F 3 "" H 6050 3850 60  0000 C CNN
	1    6050 3850
	1    0    0    -1  
$EndComp
$Comp
L XT60 P3
U 1 1 5A7531DC
P 6550 3850
F 0 "P3" H 6550 4000 50  0000 C CNN
F 1 "XT60" H 6550 3700 50  0000 C CNN
F 2 "zeabus:XT60" H 6550 3850 60  0001 C CNN
F 3 "" H 6550 3850 60  0000 C CNN
	1    6550 3850
	1    0    0    -1  
$EndComp
$Comp
L XT60 P4
U 1 1 5A75320F
P 7050 3850
F 0 "P4" H 7050 4000 50  0000 C CNN
F 1 "XT60" H 7050 3700 50  0000 C CNN
F 2 "zeabus:XT60" H 7050 3850 60  0001 C CNN
F 3 "" H 7050 3850 60  0000 C CNN
	1    7050 3850
	1    0    0    -1  
$EndComp
$EndSCHEMATC
