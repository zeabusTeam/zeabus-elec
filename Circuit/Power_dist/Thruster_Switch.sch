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
Sheet 5 10
Title "Thruster Switch"
Date "2017-04-06"
Rev "1.0.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L XT60 P1
U 1 1 58E019DD
P 6850 4300
AR Path="/58E52973/58E019DD" Ref="P1"  Part="1" 
AR Path="/58E62AD5/58E019DD" Ref="P7"  Part="1" 
AR Path="/58E0048C/58E019DD" Ref="P11"  Part="1" 
F 0 "P11" H 6850 4450 50  0000 C CNN
F 1 "XT60" H 6850 4150 50  0000 C CNN
F 2 "zeabus:XT60" H 6950 4050 60  0001 C CNN
F 3 "" H 6850 4300 60  0000 C CNN
	1    6850 4300
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 58E01A68
P 6250 4150
AR Path="/58E52973/58E01A68" Ref="R3"  Part="1" 
AR Path="/58E62AD5/58E01A68" Ref="R12"  Part="1" 
AR Path="/58E0048C/58E01A68" Ref="R15"  Part="1" 
F 0 "R15" V 6330 4150 50  0000 C CNN
F 1 "10k" V 6250 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6180 4150 50  0001 C CNN
F 3 "" H 6250 4150 50  0001 C CNN
	1    6250 4150
	1    0    0    -1  
$EndComp
$Comp
L LED_Small D2
U 1 1 58E01A8B
P 6250 4550
AR Path="/58E52973/58E01A8B" Ref="D2"  Part="1" 
AR Path="/58E62AD5/58E01A8B" Ref="D8"  Part="1" 
AR Path="/58E0048C/58E01A8B" Ref="D10"  Part="1" 
F 0 "D10" H 6200 4675 50  0000 L CNN
F 1 "LED_Small" H 6075 4450 50  0000 L CNN
F 2 "LEDs:LED_0603_HandSoldering" V 6250 4550 50  0001 C CNN
F 3 "" V 6250 4550 50  0001 C CNN
	1    6250 4550
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 58E01FA8
P 5550 2500
AR Path="/58E52973/58E01FA8" Ref="R1"  Part="1" 
AR Path="/58E62AD5/58E01FA8" Ref="R10"  Part="1" 
AR Path="/58E0048C/58E01FA8" Ref="R13"  Part="1" 
F 0 "R13" V 5630 2500 50  0000 C CNN
F 1 "10k" V 5550 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5480 2500 50  0001 C CNN
F 3 "" H 5550 2500 50  0001 C CNN
	1    5550 2500
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 58E01FFB
P 5550 3050
AR Path="/58E52973/58E01FFB" Ref="R2"  Part="1" 
AR Path="/58E62AD5/58E01FFB" Ref="R11"  Part="1" 
AR Path="/58E0048C/58E01FFB" Ref="R14"  Part="1" 
F 0 "R14" V 5630 3050 50  0000 C CNN
F 1 "2.2k" V 5550 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5480 3050 50  0001 C CNN
F 3 "" H 5550 3050 50  0001 C CNN
	1    5550 3050
	0    1    1    0   
$EndComp
Text HLabel 3700 2300 0    60   Input ~ 0
+Power
Text HLabel 3700 3050 0    60   Input ~ 0
Software_Switch
Wire Wire Line
	6650 3850 6650 4250
Connection ~ 6250 3850
Wire Wire Line
	6650 4750 6650 4350
Wire Wire Line
	5700 2500 6250 2500
Connection ~ 6250 2500
Wire Wire Line
	6250 2300 6250 2850
Wire Wire Line
	5700 3050 5950 3050
Wire Wire Line
	3700 3050 5400 3050
Wire Wire Line
	5250 2500 5400 2500
Connection ~ 5250 3050
Wire Wire Line
	6250 4450 6250 4300
Wire Wire Line
	6250 4650 6250 4800
Wire Wire Line
	6250 3250 6250 4000
$Comp
L Q_PMOS_GDS Q1
U 1 1 58E51C6A
P 6150 3050
AR Path="/58E52973/58E51C6A" Ref="Q1"  Part="1" 
AR Path="/58E62AD5/58E51C6A" Ref="Q4"  Part="1" 
AR Path="/58E0048C/58E51C6A" Ref="Q5"  Part="1" 
F 0 "Q5" H 6350 3100 50  0000 L CNN
F 1 "IPD90P03P4L" H 6350 3000 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 6350 3150 50  0001 C CNN
F 3 "" H 6150 3050 50  0001 C CNN
	1    6150 3050
	1    0    0    1   
$EndComp
$Comp
L D_Zener D1
U 1 1 58E6427A
P 5850 2750
AR Path="/58E52973/58E6427A" Ref="D1"  Part="1" 
AR Path="/58E62AD5/58E6427A" Ref="D7"  Part="1" 
AR Path="/58E0048C/58E6427A" Ref="D9"  Part="1" 
F 0 "D9" H 5850 2850 50  0000 C CNN
F 1 "BZT52c12s" H 5850 2650 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 5850 2750 50  0001 C CNN
F 3 "" H 5850 2750 50  0001 C CNN
	1    5850 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	5850 2600 5850 2500
Connection ~ 5850 2500
Wire Wire Line
	5850 2900 5850 3050
Connection ~ 5850 3050
Connection ~ 6250 4750
Wire Wire Line
	5250 3050 5250 2500
Wire Wire Line
	3700 2300 7250 2300
Wire Wire Line
	6250 4750 6650 4750
Wire Wire Line
	6650 3850 6250 3850
$Comp
L GNDPWR #PWR?
U 1 1 5B5DEA3D
P 6250 4800
AR Path="/58E0048C/5B5DEA3D" Ref="#PWR?"  Part="1" 
AR Path="/58E52973/5B5DEA3D" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6250 4600 50  0001 C CNN
F 1 "GNDPWR" H 6250 4670 50  0000 C CNN
F 2 "" H 6250 4750 50  0001 C CNN
F 3 "" H 6250 4750 50  0001 C CNN
	1    6250 4800
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5B500BB4
P 7250 2725
F 0 "C?" H 7275 2825 50  0000 L CNN
F 1 "470uF" H 7275 2625 50  0000 L CNN
F 2 "" H 7288 2575 50  0001 C CNN
F 3 "" H 7250 2725 50  0001 C CNN
	1    7250 2725
	1    0    0    -1  
$EndComp
Connection ~ 6250 2300
$Comp
L GNDPWR #PWR?
U 1 1 5B500D71
P 7250 2975
F 0 "#PWR?" H 7250 2775 50  0001 C CNN
F 1 "GNDPWR" H 7250 2845 50  0000 C CNN
F 2 "" H 7250 2925 50  0001 C CNN
F 3 "" H 7250 2925 50  0001 C CNN
	1    7250 2975
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 2300 7250 2575
Wire Wire Line
	7250 2975 7250 2875
$EndSCHEMATC
