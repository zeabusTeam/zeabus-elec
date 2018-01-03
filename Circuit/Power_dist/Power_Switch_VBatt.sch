EESchema Schematic File Version 2
LIBS:Power_Distributor-rescue
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
Sheet 3 8
Title "DVL and Sonar Switch"
Date "2017-04-06"
Rev "1.0.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Q_PMOS_GDS Q2
U 1 1 58E52089
P 6350 3300
AR Path="/58E51F99/58E52089" Ref="Q2"  Part="1" 
AR Path="/58E527AF/58E52089" Ref="Q3"  Part="1" 
F 0 "Q2" H 6550 3350 50  0000 L CNN
F 1 "IPD90P03P4L" H 6550 3250 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 6550 3400 50  0001 C CNN
F 3 "" H 6350 3300 50  0001 C CNN
	1    6350 3300
	1    0    0    1   
$EndComp
$Comp
L R R5
U 1 1 58E5219E
P 5850 3300
AR Path="/58E51F99/58E5219E" Ref="R5"  Part="1" 
AR Path="/58E527AF/58E5219E" Ref="R8"  Part="1" 
F 0 "R5" V 5930 3300 50  0000 C CNN
F 1 "2.2k" V 5850 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5780 3300 50  0001 C CNN
F 3 "" H 5850 3300 50  0001 C CNN
	1    5850 3300
	0    1    1    0   
$EndComp
$Comp
L LED_Small D4
U 1 1 58E52234
P 6450 4550
AR Path="/58E51F99/58E52234" Ref="D4"  Part="1" 
AR Path="/58E527AF/58E52234" Ref="D6"  Part="1" 
F 0 "D4" H 6400 4675 50  0000 L CNN
F 1 "LED_Small" H 6275 4450 50  0000 L CNN
F 2 "LEDs:LED_0603_HandSoldering" V 6450 4550 50  0001 C CNN
F 3 "" V 6450 4550 50  0001 C CNN
	1    6450 4550
	0    -1   -1   0   
$EndComp
$Comp
L R R4
U 1 1 58E5234D
P 5850 2700
AR Path="/58E51F99/58E5234D" Ref="R4"  Part="1" 
AR Path="/58E527AF/58E5234D" Ref="R7"  Part="1" 
F 0 "R4" V 5930 2700 50  0000 C CNN
F 1 "10k" V 5850 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5780 2700 50  0001 C CNN
F 3 "" H 5850 2700 50  0001 C CNN
	1    5850 2700
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 58E52389
P 6450 4150
AR Path="/58E51F99/58E52389" Ref="R6"  Part="1" 
AR Path="/58E527AF/58E52389" Ref="R9"  Part="1" 
F 0 "R6" V 6530 4150 50  0000 C CNN
F 1 "10k" V 6450 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6380 4150 50  0001 C CNN
F 3 "" H 6450 4150 50  0001 C CNN
	1    6450 4150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR027
U 1 1 58E527F8
P 6450 4950
AR Path="/58E51F99/58E527F8" Ref="#PWR027"  Part="1" 
AR Path="/58E527AF/58E527F8" Ref="#PWR028"  Part="1" 
F 0 "#PWR027" H 6450 4700 50  0001 C CNN
F 1 "GND" H 6450 4800 50  0000 C CNN
F 2 "" H 6450 4950 50  0001 C CNN
F 3 "" H 6450 4950 50  0001 C CNN
	1    6450 4950
	1    0    0    -1  
$EndComp
Text HLabel 3650 2450 0    60   Input ~ 0
+Power
Text HLabel 3650 3300 0    60   Input ~ 0
Software_Switch
Wire Wire Line
	6000 3300 6150 3300
Wire Wire Line
	3650 3300 5700 3300
Connection ~ 5550 3300
Wire Wire Line
	6450 2450 6450 3100
$Comp
L D_Zener D3
U 1 1 58E648CE
P 6100 3000
AR Path="/58E51F99/58E648CE" Ref="D3"  Part="1" 
AR Path="/58E527AF/58E648CE" Ref="D5"  Part="1" 
F 0 "D3" H 6100 3100 50  0000 C CNN
F 1 "BZX585-C12" H 6100 2900 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 6100 3000 50  0001 C CNN
F 3 "" H 6100 3000 50  0001 C CNN
	1    6100 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	5550 2700 5700 2700
Wire Wire Line
	6000 2700 6450 2700
Wire Wire Line
	6100 2850 6100 2700
Connection ~ 6100 2700
Wire Wire Line
	6100 3150 6100 3300
Connection ~ 6100 3300
Wire Wire Line
	3650 2450 6450 2450
Connection ~ 6450 2700
$Comp
L Fuse F2
U 1 1 58E651FD
P 6450 3750
AR Path="/58E51F99/58E651FD" Ref="F2"  Part="1" 
AR Path="/58E527AF/58E651FD" Ref="F3"  Part="1" 
F 0 "F2" V 6530 3750 50  0000 C CNN
F 1 "Fuse" V 6375 3750 50  0000 C CNN
F 2 "zeabus:FUSE_AUTO" V 6380 3750 50  0001 C CNN
F 3 "" H 6450 3750 50  0001 C CNN
	1    6450 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3500 6450 3600
Wire Wire Line
	6450 3900 6450 4000
Wire Wire Line
	6450 4300 6450 4450
Wire Wire Line
	6450 4650 6450 4950
Wire Wire Line
	6850 4850 6850 4300
Connection ~ 6450 4850
Wire Wire Line
	6450 3950 6850 3950
Wire Wire Line
	6850 3950 6850 4200
Connection ~ 6450 3950
Wire Wire Line
	5550 2700 5550 3300
Wire Wire Line
	6850 4850 6450 4850
$Comp
L XT30 P2
U 1 1 5A39572B
P 7050 4250
AR Path="/58E51F99/5A39572B" Ref="P2"  Part="1" 
AR Path="/58E527AF/5A39572B" Ref="P3"  Part="1" 
F 0 "P2" H 7050 4400 50  0000 C CNN
F 1 "XT30" H 7050 4100 50  0000 C CNN
F 2 "zeabus:XT30" H 7050 4250 60  0000 C CNN
F 3 "" H 7050 4250 60  0000 C CNN
	1    7050 4250
	1    0    0    -1  
$EndComp
$EndSCHEMATC
