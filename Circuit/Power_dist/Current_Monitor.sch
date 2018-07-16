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
Sheet 9 9
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
L R R?
U 1 1 5B4C1590
P 3725 2825
F 0 "R?" V 3805 2825 50  0000 C CNN
F 1 "2m" V 3725 2825 50  0000 C CNN
F 2 "Resistors_SMD:R_2512_HandSoldering" V 3655 2825 50  0001 C CNN
F 3 "" H 3725 2825 50  0001 C CNN
	1    3725 2825
	0    1    1    0   
$EndComp
Text HLabel 3050 2675 0    60   UnSpc ~ 0
Current_IN
Text HLabel 6300 2675 2    60   UnSpc ~ 0
Current_OUT
Wire Wire Line
	3475 2525 3575 2525
Wire Wire Line
	3475 2525 3475 3750
Wire Wire Line
	3975 2525 3975 4000
Wire Wire Line
	3975 2675 6300 2675
Connection ~ 3975 2675
Wire Wire Line
	3050 2675 3475 2675
Connection ~ 3475 2675
Wire Wire Line
	3475 3750 4225 3750
Connection ~ 3475 2825
Wire Wire Line
	3975 4000 4225 4000
Connection ~ 3975 2825
Wire Wire Line
	3875 2525 3975 2525
Wire Wire Line
	3575 2825 3475 2825
Wire Wire Line
	3875 2825 3975 2825
$Comp
L INA240 U?
U 1 1 5B4C345B
P 4525 3875
F 0 "U?" H 4675 4000 50  0000 L CNN
F 1 "INA240" H 4675 3750 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 4525 4275 50  0001 C CNN
F 3 "" H 4625 3875 50  0001 C CNN
	1    4525 3875
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B4C3925
P 4525 4350
F 0 "#PWR?" H 4525 4100 50  0001 C CNN
F 1 "GND" H 4525 4200 50  0000 C CNN
F 2 "" H 4525 4350 50  0001 C CNN
F 3 "" H 4525 4350 50  0001 C CNN
	1    4525 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4525 4175 4525 4350
Wire Wire Line
	4425 4175 4425 4275
Wire Wire Line
	4425 4275 4525 4275
Connection ~ 4525 4275
$Comp
L LM4041CF U?
U 1 1 5B4C7EB6
P 5600 4700
F 0 "U?" H 5325 4900 50  0000 C CNN
F 1 "LM4041CF" H 5575 4900 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 5600 5000 50  0001 C CIN
F 3 "" H 5325 4825 50  0001 C CNN
	1    5600 4700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B4C8001
P 3725 2525
F 0 "R?" V 3805 2525 50  0000 C CNN
F 1 "2m" V 3725 2525 50  0000 C CNN
F 2 "Resistors_SMD:R_2512_HandSoldering" V 3655 2525 50  0001 C CNN
F 3 "" H 3725 2525 50  0001 C CNN
	1    3725 2525
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5B4C80AE
P 4725 4700
F 0 "R?" V 4805 4700 50  0000 C CNN
F 1 "1.5k" V 4725 4700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4655 4700 50  0001 C CNN
F 3 "" H 4725 4700 50  0001 C CNN
	1    4725 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	3250 4700 4575 4700
Wire Wire Line
	3250 2675 3250 4700
Connection ~ 3250 2675
Wire Wire Line
	4625 4175 4625 4275
Wire Wire Line
	4625 4275 5025 4275
Wire Wire Line
	5025 3450 5025 4700
Wire Wire Line
	4875 4700 5175 4700
Connection ~ 5025 4700
Wire Wire Line
	5025 3450 4625 3450
Wire Wire Line
	4625 3450 4625 3575
Connection ~ 5025 4275
Wire Wire Line
	4925 3875 6300 3875
Text HLabel 6300 3875 2    60   Output ~ 0
OUT
$Comp
L GND #PWR?
U 1 1 5B4CDA84
P 6075 4875
F 0 "#PWR?" H 6075 4625 50  0001 C CNN
F 1 "GND" H 6075 4725 50  0000 C CNN
F 2 "" H 6075 4875 50  0001 C CNN
F 3 "" H 6075 4875 50  0001 C CNN
	1    6075 4875
	1    0    0    -1  
$EndComp
Wire Wire Line
	6025 4700 6075 4700
Wire Wire Line
	6075 4700 6075 4875
$Comp
L C_Small C?
U 1 1 5B4D120C
P 4150 3325
F 0 "C?" H 4160 3395 50  0000 L CNN
F 1 "0.1uF" H 4160 3245 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4150 3325 50  0001 C CNN
F 3 "" H 4150 3325 50  0001 C CNN
	1    4150 3325
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B4D19FF
P 4150 3500
F 0 "#PWR?" H 4150 3250 50  0001 C CNN
F 1 "GND" H 4150 3350 50  0000 C CNN
F 2 "" H 4150 3500 50  0001 C CNN
F 3 "" H 4150 3500 50  0001 C CNN
	1    4150 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 3500 4150 3425
Wire Wire Line
	3050 3125 4525 3125
Wire Wire Line
	4150 3125 4150 3225
Wire Wire Line
	4525 3125 4525 3575
Connection ~ 4150 3125
Text HLabel 3050 3125 0    60   UnSpc ~ 0
+5V_PWR
$EndSCHEMATC
