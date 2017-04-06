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
Sheet 1 13
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 9000 4450 900  400 
U 58C990D6
F0 "USB_N_Ethernet" 60
F1 "Power_5V.sch" 60
F2 "+Power" I L 9000 4550 60 
$EndSheet
$Sheet
S 9000 3050 900  400 
U 58C998BD
F0 "Lamp_Switch" 60
F1 "Lamp_Switch.sch" 60
F2 "+Power" I L 9000 3150 60 
F3 "Software_Switch" I L 9000 3250 60 
$EndSheet
$Sheet
S 9000 3750 900  400 
U 58DFD88E
F0 "DSP" 60
F1 "Power_Switch_5V.sch" 60
F2 "+Power" I L 9000 3850 60 
F3 "Software_Switch" I L 9000 3950 60 
$EndSheet
$Sheet
S 5250 4450 900  400 
U 58DFDFD6
F0 "NUC" 60
F1 "Power_12V.sch" 60
F2 "+Power" I R 6150 4550 60 
$EndSheet
$Sheet
S 5250 2350 900  400 
U 58E52973
F0 "Thruster2" 60
F1 "Thruster_Switch.sch" 60
F2 "+Power" I R 6150 2450 60 
F3 "Software_Switch" I R 6150 2550 60 
F4 "Hardware_Switch" I R 6150 2650 60 
$EndSheet
$Sheet
S 5250 3050 900  400 
U 58E51F99
F0 "DVL" 60
F1 "Power_Switch_VBatt.sch" 60
F2 "+Power" I R 6150 3150 60 
F3 "Software_Switch" I R 6150 3250 60 
F4 "Hardware_Switch" I R 6150 3350 60 
$EndSheet
$Sheet
S 5250 3750 900  400 
U 58E527AF
F0 "Imagine_Sonar" 60
F1 "Power_Switch_VBatt.sch" 60
F2 "+Power" I R 6150 3850 60 
F3 "Software_Switch" I R 6150 3950 60 
F4 "Hardware_Switch" I R 6150 4050 60 
$EndSheet
$Sheet
S 9000 1650 900  400 
U 58E62AD5
F0 "Thruster_Switch_Spare1" 60
F1 "Thruster_Switch.sch" 60
F2 "+Power" I L 9000 1750 60 
F3 "Software_Switch" I L 9000 1850 60 
F4 "Hardware_Switch" I L 9000 1950 60 
$EndSheet
$Sheet
S 9000 2350 900  400 
U 58E6D43E
F0 "Thruster_Switch_Spare2" 60
F1 "Thruster_Switch.sch" 60
F2 "+Power" I L 9000 2450 60 
F3 "Software_Switch" I L 9000 2550 60 
F4 "Hardware_Switch" I L 9000 2650 60 
$EndSheet
$Comp
L +BATT #PWR?
U 1 1 58E61C00
P 4500 2300
F 0 "#PWR?" H 4500 2150 50  0001 C CNN
F 1 "+BATT" H 4500 2440 50  0000 C CNN
F 2 "" H 4500 2300 50  0001 C CNN
F 3 "" H 4500 2300 50  0001 C CNN
	1    4500 2300
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 J?
U 1 1 58E6ABEA
P 2450 3250
F 0 "J?" H 2450 3400 50  0000 C CNN
F 1 "Thruster_Off_Switch" V 2550 3250 50  0000 C CNN
F 2 "" H 2450 3250 50  0001 C CNN
F 3 "" H 2450 3250 50  0001 C CNN
	1    2450 3250
	-1   0    0    1   
$EndComp
Text Label 3300 3200 2    60   ~ 0
Thruster_Off
$Comp
L GND #PWR?
U 1 1 58E6E6F3
P 2750 3450
F 0 "#PWR?" H 2750 3200 50  0001 C CNN
F 1 "GND" H 2750 3300 50  0000 C CNN
F 2 "" H 2750 3450 50  0001 C CNN
F 3 "" H 2750 3450 50  0001 C CNN
	1    2750 3450
	1    0    0    -1  
$EndComp
Text Label 6750 1950 2    60   ~ 0
Thruster_Off
$Sheet
S 5250 1650 900  400 
U 58E0048C
F0 "Thruster1" 60
F1 "Thruster_Switch.sch" 60
F2 "+Power" I R 6150 1750 60 
F3 "Software_Switch" I R 6150 1850 60 
F4 "Hardware_Switch" I R 6150 1950 60 
$EndSheet
Text Label 6750 2650 2    60   ~ 0
Thruster_Off
Wire Wire Line
	2650 3300 2750 3300
Wire Wire Line
	2750 3300 2750 3450
Wire Wire Line
	3300 3200 2650 3200
Text Label 8400 1950 0    60   ~ 0
Thruster_Off
Wire Wire Line
	8400 1950 9000 1950
Wire Wire Line
	6750 1950 6150 1950
Wire Wire Line
	6750 2650 6150 2650
Text Label 8400 2650 0    60   ~ 0
Thruster_Off
Wire Wire Line
	8400 2650 9000 2650
$Comp
L +BATT #PWR?
U 1 1 58EB597A
P 6300 1650
F 0 "#PWR?" H 6300 1500 50  0001 C CNN
F 1 "+BATT" H 6300 1790 50  0000 C CNN
F 2 "" H 6300 1650 50  0001 C CNN
F 3 "" H 6300 1650 50  0001 C CNN
	1    6300 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 1650 6300 1750
Wire Wire Line
	6300 1750 6150 1750
$Comp
L +BATT #PWR?
U 1 1 58EB69B2
P 6300 2350
F 0 "#PWR?" H 6300 2200 50  0001 C CNN
F 1 "+BATT" H 6300 2490 50  0000 C CNN
F 2 "" H 6300 2350 50  0001 C CNN
F 3 "" H 6300 2350 50  0001 C CNN
	1    6300 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 2350 6300 2450
Wire Wire Line
	6300 2450 6150 2450
$Comp
L +BATT #PWR?
U 1 1 58EB9A19
P 6300 3050
F 0 "#PWR?" H 6300 2900 50  0001 C CNN
F 1 "+BATT" H 6300 3190 50  0000 C CNN
F 2 "" H 6300 3050 50  0001 C CNN
F 3 "" H 6300 3050 50  0001 C CNN
	1    6300 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3050 6300 3150
Wire Wire Line
	6300 3150 6150 3150
$Comp
L +BATT #PWR?
U 1 1 58EBAA49
P 6300 3750
F 0 "#PWR?" H 6300 3600 50  0001 C CNN
F 1 "+BATT" H 6300 3890 50  0000 C CNN
F 2 "" H 6300 3750 50  0001 C CNN
F 3 "" H 6300 3750 50  0001 C CNN
	1    6300 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 3850 6300 3850
Wire Wire Line
	6300 3850 6300 3750
$Comp
L +BATT #PWR?
U 1 1 58EBD7FB
P 6300 4450
F 0 "#PWR?" H 6300 4300 50  0001 C CNN
F 1 "+BATT" H 6300 4590 50  0000 C CNN
F 2 "" H 6300 4450 50  0001 C CNN
F 3 "" H 6300 4450 50  0001 C CNN
	1    6300 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4550 6300 4550
Wire Wire Line
	6300 4550 6300 4450
$Comp
L +BATT #PWR?
U 1 1 58EC0437
P 8850 1650
F 0 "#PWR?" H 8850 1500 50  0001 C CNN
F 1 "+BATT" H 8850 1790 50  0000 C CNN
F 2 "" H 8850 1650 50  0001 C CNN
F 3 "" H 8850 1650 50  0001 C CNN
	1    8850 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 1650 8850 1750
Wire Wire Line
	8850 1750 9000 1750
$Comp
L +BATT #PWR?
U 1 1 58ECB208
P 8850 2350
F 0 "#PWR?" H 8850 2200 50  0001 C CNN
F 1 "+BATT" H 8850 2490 50  0000 C CNN
F 2 "" H 8850 2350 50  0001 C CNN
F 3 "" H 8850 2350 50  0001 C CNN
	1    8850 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 2350 8850 2450
Wire Wire Line
	8850 2450 9000 2450
$Comp
L +BATT #PWR?
U 1 1 58ECC2C0
P 8850 3050
F 0 "#PWR?" H 8850 2900 50  0001 C CNN
F 1 "+BATT" H 8850 3190 50  0000 C CNN
F 2 "" H 8850 3050 50  0001 C CNN
F 3 "" H 8850 3050 50  0001 C CNN
	1    8850 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 3050 8850 3150
Wire Wire Line
	8850 3150 9000 3150
$Comp
L +BATT #PWR?
U 1 1 58ECD1FA
P 8850 3750
F 0 "#PWR?" H 8850 3600 50  0001 C CNN
F 1 "+BATT" H 8850 3890 50  0000 C CNN
F 2 "" H 8850 3750 50  0001 C CNN
F 3 "" H 8850 3750 50  0001 C CNN
	1    8850 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 3750 8850 3850
Wire Wire Line
	8850 3850 9000 3850
$Comp
L +BATT #PWR?
U 1 1 58ECE636
P 8850 4450
F 0 "#PWR?" H 8850 4300 50  0001 C CNN
F 1 "+BATT" H 8850 4590 50  0000 C CNN
F 2 "" H 8850 4450 50  0001 C CNN
F 3 "" H 8850 4450 50  0001 C CNN
	1    8850 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 4450 8850 4550
Wire Wire Line
	8850 4550 9000 4550
$Sheet
S 7150 1650 1000 3200
U 58E54EC3
F0 "Power_Controller" 60
F1 "Power-Controller.sch" 60
F2 "SW1_C" I L 7150 1850 60 
F3 "SW2_C" I L 7150 2550 60 
F4 "SW3_C" I L 7150 3250 60 
F5 "SW4_C" I L 7150 3950 60 
F6 "SW5_C" I R 8150 1850 60 
F7 "SW6_C" I R 8150 2550 60 
F8 "SW7_C" I R 8150 3250 60 
F9 "SW8_C" I R 8150 3950 60 
F10 "SW1_E" O L 7150 1950 60 
F11 "SW2_E" O L 7150 2650 60 
F12 "SW3_E" O L 7150 3350 60 
F13 "SW4_E" O L 7150 4050 60 
F14 "SW5_E" O R 8150 1950 60 
F15 "SW6_E" O R 8150 2650 60 
F16 "SW7_E" O R 8150 3350 60 
F17 "SW8_E" O R 8150 4050 60 
$EndSheet
$Sheet
S 3050 2250 1050 400 
U 58E5E8B2
F0 "Power_Aggregator" 60
F1 "Power_Aggregator.sch" 60
F2 "+VBatt" U R 4100 2450 60 
$EndSheet
Wire Wire Line
	4100 2450 4500 2450
Wire Wire Line
	4500 2450 4500 2300
$EndSCHEMATC
