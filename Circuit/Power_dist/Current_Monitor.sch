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
F 2 "" V 3655 2825 50  0001 C CNN
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
	3475 2525 3475 3500
Wire Wire Line
	3975 2525 3975 3750
Wire Wire Line
	3975 2675 6300 2675
Connection ~ 3975 2675
Wire Wire Line
	3050 2675 3475 2675
Connection ~ 3475 2675
Wire Wire Line
	3475 3500 4225 3500
Connection ~ 3475 2825
Wire Wire Line
	3975 3750 4225 3750
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
P 4525 3625
F 0 "U?" H 4675 3750 50  0000 L CNN
F 1 "INA240" H 4675 3500 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 4525 4025 50  0001 C CNN
F 3 "" H 4625 3625 50  0001 C CNN
	1    4525 3625
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B4C3925
P 4525 4100
F 0 "#PWR?" H 4525 3850 50  0001 C CNN
F 1 "GND" H 4525 3950 50  0000 C CNN
F 2 "" H 4525 4100 50  0001 C CNN
F 3 "" H 4525 4100 50  0001 C CNN
	1    4525 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4525 3925 4525 4100
Wire Wire Line
	4425 3925 4425 4025
Wire Wire Line
	4425 4025 4525 4025
Connection ~ 4525 4025
$Comp
L LM4041CF U?
U 1 1 5B4C7EB6
P 5600 4450
F 0 "U?" H 5325 4650 50  0000 C CNN
F 1 "LM4041CF" H 5575 4650 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 5600 4750 50  0001 C CIN
F 3 "" H 5325 4575 50  0001 C CNN
	1    5600 4450
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B4C8001
P 3725 2525
F 0 "R?" V 3805 2525 50  0000 C CNN
F 1 "2m" V 3725 2525 50  0000 C CNN
F 2 "" V 3655 2525 50  0001 C CNN
F 3 "" H 3725 2525 50  0001 C CNN
	1    3725 2525
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5B4C80AE
P 4725 4450
F 0 "R?" V 4805 4450 50  0000 C CNN
F 1 "1.5k" V 4725 4450 50  0000 C CNN
F 2 "" V 4655 4450 50  0001 C CNN
F 3 "" H 4725 4450 50  0001 C CNN
	1    4725 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	4575 4450 3250 4450
Wire Wire Line
	3250 4450 3250 2675
Connection ~ 3250 2675
Wire Wire Line
	4625 3925 4625 4025
Wire Wire Line
	4625 4025 5025 4025
Wire Wire Line
	5025 3200 5025 4450
Wire Wire Line
	4875 4450 5175 4450
Connection ~ 5025 4450
Wire Wire Line
	5025 3200 4625 3200
Wire Wire Line
	4625 3200 4625 3325
Connection ~ 5025 4025
Wire Wire Line
	4925 3625 6300 3625
Text HLabel 6300 3625 2    60   Output ~ 0
OUT
$Comp
L GND #PWR?
U 1 1 5B4CDA84
P 6075 4625
F 0 "#PWR?" H 6075 4375 50  0001 C CNN
F 1 "GND" H 6075 4475 50  0000 C CNN
F 2 "" H 6075 4625 50  0001 C CNN
F 3 "" H 6075 4625 50  0001 C CNN
	1    6075 4625
	1    0    0    -1  
$EndComp
Wire Wire Line
	6025 4450 6075 4450
Wire Wire Line
	6075 4450 6075 4625
$EndSCHEMATC
