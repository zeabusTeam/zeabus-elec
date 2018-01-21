EESchema Schematic File Version 2
LIBS:board-drive-rescue
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
LIBS:board-drive-cache
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
L Si8660BA-B-IS1 U2
U 1 1 5A54F001
P 3250 2500
F 0 "U2" H 3550 2600 60  0000 C CNN
F 1 "Si8660BA-B-IS1" H 3750 1500 60  0000 C CNN
F 2 "zeabus:Narrow-SOIC-16" H 3250 2500 60  0001 C CNN
F 3 "" H 3250 2500 60  0001 C CNN
	1    3250 2500
	1    0    0    -1  
$EndComp
$Comp
L Si8660BA-B-IS1 U3
U 1 1 5A54F032
P 3250 5100
F 0 "U3" H 3550 5200 60  0000 C CNN
F 1 "Si8660BA-B-IS1" H 3750 4100 60  0000 C CNN
F 2 "zeabus:Narrow-SOIC-16" H 3250 5100 60  0001 C CNN
F 3 "" H 3250 5100 60  0001 C CNN
	1    3250 5100
	1    0    0    -1  
$EndComp
Text Label 4250 2700 0    60   ~ 0
signal0
Text Label 4250 2800 0    60   ~ 0
signal1
Text Label 4250 2900 0    60   ~ 0
signal2
Text Label 4250 3000 0    60   ~ 0
signal3
Text Label 4250 3100 0    60   ~ 0
signal4
Text Label 4250 3200 0    60   ~ 0
signal5
Text Label 4250 5300 0    60   ~ 0
signal6
Text Label 4250 5400 0    60   ~ 0
signal7
Text Label 4250 5500 0    60   ~ 0
signal8
Text Label 4250 5600 0    60   ~ 0
signal9
Text Label 4250 5700 0    60   ~ 0
signal10
Text Label 4250 5800 0    60   ~ 0
signal11
$Comp
L +5V #PWR01
U 1 1 5A552B6C
P 750 3050
F 0 "#PWR01" H 750 2900 50  0001 C CNN
F 1 "+5V" H 750 3200 50  0000 C CNN
F 2 "" H 750 3050 50  0001 C CNN
F 3 "" H 750 3050 50  0001 C CNN
	1    750  3050
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR02
U 1 1 5A555BDF
P 4250 6000
F 0 "#PWR02" H 4250 5750 50  0001 C CNN
F 1 "GNDA" H 4250 5850 50  0000 C CNN
F 2 "" H 4250 6000 50  0001 C CNN
F 3 "" H 4250 6000 50  0001 C CNN
	1    4250 6000
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR03
U 1 1 5A555C61
P 4250 3400
F 0 "#PWR03" H 4250 3150 50  0001 C CNN
F 1 "GNDA" H 4250 3250 50  0000 C CNN
F 2 "" H 4250 3400 50  0001 C CNN
F 3 "" H 4250 3400 50  0001 C CNN
	1    4250 3400
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR04
U 1 1 5A563C1F
P 750 5550
F 0 "#PWR04" H 750 5300 50  0001 C CNN
F 1 "GNDD" H 750 5425 50  0000 C CNN
F 2 "" H 750 5550 50  0001 C CNN
F 3 "" H 750 5550 50  0001 C CNN
	1    750  5550
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR05
U 1 1 5A563C55
P 1050 5550
F 0 "#PWR05" H 1050 5300 50  0001 C CNN
F 1 "GNDD" H 1050 5425 50  0000 C CNN
F 2 "" H 1050 5550 50  0001 C CNN
F 3 "" H 1050 5550 50  0001 C CNN
	1    1050 5550
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR06
U 1 1 5A563C84
P 3250 3400
F 0 "#PWR06" H 3250 3150 50  0001 C CNN
F 1 "GNDD" H 3250 3275 50  0000 C CNN
F 2 "" H 3250 3400 50  0001 C CNN
F 3 "" H 3250 3400 50  0001 C CNN
	1    3250 3400
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR07
U 1 1 5A563CB3
P 3250 6000
F 0 "#PWR07" H 3250 5750 50  0001 C CNN
F 1 "GNDD" H 3250 5875 50  0000 C CNN
F 2 "" H 3250 6000 50  0001 C CNN
F 3 "" H 3250 6000 50  0001 C CNN
	1    3250 6000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR08
U 1 1 5A563EBC
P 3250 2500
F 0 "#PWR08" H 3250 2350 50  0001 C CNN
F 1 "+5V" H 3250 2650 50  0000 C CNN
F 2 "" H 3250 2500 50  0001 C CNN
F 3 "" H 3250 2500 50  0001 C CNN
	1    3250 2500
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR09
U 1 1 5A563EEB
P 3250 5100
F 0 "#PWR09" H 3250 4950 50  0001 C CNN
F 1 "+5V" H 3250 5250 50  0000 C CNN
F 2 "" H 3250 5100 50  0001 C CNN
F 3 "" H 3250 5100 50  0001 C CNN
	1    3250 5100
	1    0    0    -1  
$EndComp
$Comp
L XT30 P5
U 1 1 5A563F56
P 2800 800
F 0 "P5" H 2800 950 50  0000 C CNN
F 1 "XT30" H 2800 650 50  0000 C CNN
F 2 "zeabus:XT30" H 2800 800 60  0001 C CNN
F 3 "" H 2800 800 60  0000 C CNN
	1    2800 800 
	1    0    0    -1  
$EndComp
$Comp
L XT30 P3
U 1 1 5A563FB5
P 2100 800
F 0 "P3" H 2100 950 50  0000 C CNN
F 1 "XT30" H 2100 650 50  0000 C CNN
F 2 "zeabus:XT30" H 2100 800 60  0001 C CNN
F 3 "" H 2100 800 60  0000 C CNN
	1    2100 800 
	1    0    0    -1  
$EndComp
$Comp
L XT30 P1
U 1 1 5A564005
P 1400 800
F 0 "P1" H 1400 950 50  0000 C CNN
F 1 "XT30" H 1400 650 50  0000 C CNN
F 2 "zeabus:XT30" H 1400 800 60  0001 C CNN
F 3 "" H 1400 800 60  0000 C CNN
	1    1400 800 
	1    0    0    -1  
$EndComp
$Comp
L XT30 P8
U 1 1 5A5640DE
P 3500 1250
F 0 "P8" H 3500 1400 50  0000 C CNN
F 1 "XT30" H 3500 1100 50  0000 C CNN
F 2 "zeabus:XT30" H 3500 1250 60  0001 C CNN
F 3 "" H 3500 1250 60  0000 C CNN
	1    3500 1250
	1    0    0    -1  
$EndComp
$Comp
L XT30 P6
U 1 1 5A56412C
P 2800 1250
F 0 "P6" H 2800 1400 50  0000 C CNN
F 1 "XT30" H 2800 1100 50  0000 C CNN
F 2 "zeabus:XT30" H 2800 1250 60  0001 C CNN
F 3 "" H 2800 1250 60  0000 C CNN
	1    2800 1250
	1    0    0    -1  
$EndComp
$Comp
L XT30 P4
U 1 1 5A564181
P 2100 1250
F 0 "P4" H 2100 1400 50  0000 C CNN
F 1 "XT30" H 2100 1100 50  0000 C CNN
F 2 "zeabus:XT30" H 2100 1250 60  0001 C CNN
F 3 "" H 2100 1250 60  0000 C CNN
	1    2100 1250
	1    0    0    -1  
$EndComp
$Comp
L XT30 P2
U 1 1 5A5641D5
P 1400 1250
F 0 "P2" H 1400 1400 50  0000 C CNN
F 1 "XT30" H 1400 1100 50  0000 C CNN
F 2 "zeabus:XT30" H 1400 1250 60  0001 C CNN
F 3 "" H 1400 1250 60  0000 C CNN
	1    1400 1250
	1    0    0    -1  
$EndComp
$Comp
L XT30 P7
U 1 1 5A56478A
P 3500 800
F 0 "P7" H 3500 950 50  0000 C CNN
F 1 "XT30" H 3500 650 50  0000 C CNN
F 2 "zeabus:XT30" H 3500 800 60  0001 C CNN
F 3 "" H 3500 800 60  0000 C CNN
	1    3500 800 
	1    0    0    -1  
$EndComp
Text Label 900  750  0    60   ~ 0
OUTA_0
Text Label 900  850  0    60   ~ 0
OUTB_0
Text Label 1600 750  0    60   ~ 0
OUTA_1
Text Label 1600 850  0    60   ~ 0
OUTB_1
Text Label 2300 750  0    60   ~ 0
OUTA_2
Text Label 2300 850  0    60   ~ 0
OUTB_2
Text Label 3000 750  0    60   ~ 0
OUTA_3
Text Label 3000 850  0    60   ~ 0
OUTB_3
Text Label 900  1200 0    60   ~ 0
OUTA_4
Text Label 900  1300 0    60   ~ 0
OUTB_4
Text Label 1600 1200 0    60   ~ 0
OUTA_5
Text Label 1600 1300 0    60   ~ 0
OUTB_5
Text Label 2300 1200 0    60   ~ 0
OUTA_6
Text Label 2300 1300 0    60   ~ 0
OUTB_6
Text Label 3000 1200 0    60   ~ 0
OUTA_7
Text Label 3000 1300 0    60   ~ 0
OUTB_7
NoConn ~ 1300 3700
NoConn ~ 1300 4100
NoConn ~ 1300 4200
NoConn ~ 1300 4300
NoConn ~ 1300 4400
NoConn ~ 1300 4500
$Comp
L XT60 P9
U 1 1 5A575EEE
P 2450 7050
F 0 "P9" H 2450 7200 50  0000 C CNN
F 1 "XT60" H 2450 6900 50  0000 C CNN
F 2 "zeabus:XT60" H 2450 7050 60  0001 C CNN
F 3 "" H 2450 7050 60  0000 C CNN
	1    2450 7050
	1    0    0    -1  
$EndComp
$Comp
L XT60 P10
U 1 1 5A575F6B
P 3350 7050
F 0 "P10" H 3350 7200 50  0000 C CNN
F 1 "XT60" H 3350 6900 50  0000 C CNN
F 2 "zeabus:XT60" H 3350 7050 60  0001 C CNN
F 3 "" H 3350 7050 60  0000 C CNN
	1    3350 7050
	1    0    0    -1  
$EndComp
Text Label 1850 7000 0    60   ~ 0
BATTIN_0
Text Label 1750 7100 0    60   ~ 0
BATTGND
Text Label 2750 7000 0    60   ~ 0
BATTIN_1
Text Label 2650 7100 0    60   ~ 0
BATTGND
$Comp
L PowerDist_WiretoBoard U11
U 1 1 5A5AF0D7
P 7150 5750
F 0 "U11" H 6700 5850 60  0000 C CNN
F 1 "PowerDist_WiretoBoard" H 7150 5750 60  0000 C CNN
F 2 "zeabus:PowerDist_WiretoBoard" H 7150 5750 60  0001 C CNN
F 3 "" H 7150 5750 60  0001 C CNN
	1    7150 5750
	1    0    0    -1  
$EndComp
$Comp
L Pololu_SimpleMotorController24v23_withnoPowerDist U7
U 1 1 5A5AF0D1
P 5900 5950
F 0 "U7" H 5500 6850 60  0000 C CNN
F 1 "Pololu_SimpleMotorController24v23_withnoPowerDist" H 6250 6750 60  0000 C CNN
F 2 "zeabus:pololu_motorcontroller24v23" H 6250 6750 60  0001 C CNN
F 3 "" H 6250 6750 60  0001 C CNN
	1    5900 5950
	1    0    0    -1  
$EndComp
$Comp
L PowerDist_WiretoBoard U10
U 1 1 5A5AF036
P 7150 4250
F 0 "U10" H 6700 4350 60  0000 C CNN
F 1 "PowerDist_WiretoBoard" H 7150 4250 60  0000 C CNN
F 2 "zeabus:PowerDist_WiretoBoard" H 7150 4250 60  0001 C CNN
F 3 "" H 7150 4250 60  0001 C CNN
	1    7150 4250
	1    0    0    -1  
$EndComp
$Comp
L Pololu_SimpleMotorController24v23_withnoPowerDist U6
U 1 1 5A5AF030
P 5900 4450
F 0 "U6" H 5500 5350 60  0000 C CNN
F 1 "Pololu_SimpleMotorController24v23_withnoPowerDist" H 6250 5250 60  0000 C CNN
F 2 "zeabus:pololu_motorcontroller24v23" H 6250 5250 60  0001 C CNN
F 3 "" H 6250 5250 60  0001 C CNN
	1    5900 4450
	1    0    0    -1  
$EndComp
$Comp
L PowerDist_WiretoBoard U9
U 1 1 5A5AEF93
P 7150 2700
F 0 "U9" H 6700 2800 60  0000 C CNN
F 1 "PowerDist_WiretoBoard" H 7150 2700 60  0000 C CNN
F 2 "zeabus:PowerDist_WiretoBoard" H 7150 2700 60  0001 C CNN
F 3 "" H 7150 2700 60  0001 C CNN
	1    7150 2700
	1    0    0    -1  
$EndComp
$Comp
L Pololu_SimpleMotorController24v23_withnoPowerDist U5
U 1 1 5A5AEF8D
P 5900 2950
F 0 "U5" H 5500 3850 60  0000 C CNN
F 1 "Pololu_SimpleMotorController24v23_withnoPowerDist" H 6250 3750 60  0000 C CNN
F 2 "zeabus:pololu_motorcontroller24v23" H 6250 3750 60  0001 C CNN
F 3 "" H 6250 3750 60  0001 C CNN
	1    5900 2950
	1    0    0    -1  
$EndComp
$Comp
L PowerDist_WiretoBoard U8
U 1 1 5A5AE56A
P 7150 1250
F 0 "U8" H 6700 1350 60  0000 C CNN
F 1 "PowerDist_WiretoBoard" H 7150 1250 60  0000 C CNN
F 2 "zeabus:PowerDist_WiretoBoard" H 7150 1250 60  0001 C CNN
F 3 "" H 7150 1250 60  0001 C CNN
	1    7150 1250
	1    0    0    -1  
$EndComp
$Comp
L Pololu_SimpleMotorController24v23_withnoPowerDist U4
U 1 1 5A5AE339
P 5900 1450
F 0 "U4" H 5500 2350 60  0000 C CNN
F 1 "Pololu_SimpleMotorController24v23_withnoPowerDist" H 6250 2250 60  0000 C CNN
F 2 "zeabus:pololu_motorcontroller24v23" H 6250 2250 60  0001 C CNN
F 3 "" H 6250 2250 60  0001 C CNN
	1    5900 1450
	1    0    0    -1  
$EndComp
$Comp
L PowerDist_WiretoBoard U19
U 1 1 5A5AFE8D
P 10150 5750
F 0 "U19" H 9700 5850 60  0000 C CNN
F 1 "PowerDist_WiretoBoard" H 10150 5750 60  0000 C CNN
F 2 "zeabus:PowerDist_WiretoBoard" H 10150 5750 60  0001 C CNN
F 3 "" H 10150 5750 60  0001 C CNN
	1    10150 5750
	1    0    0    -1  
$EndComp
$Comp
L Pololu_SimpleMotorController24v23_withnoPowerDist U15
U 1 1 5A5AFE93
P 8900 5950
F 0 "U15" H 8500 6850 60  0000 C CNN
F 1 "Pololu_SimpleMotorController24v23_withnoPowerDist" H 9250 6750 60  0000 C CNN
F 2 "zeabus:pololu_motorcontroller24v23" H 9250 6750 60  0001 C CNN
F 3 "" H 9250 6750 60  0001 C CNN
	1    8900 5950
	1    0    0    -1  
$EndComp
$Comp
L PowerDist_WiretoBoard U18
U 1 1 5A5AFE99
P 10150 4250
F 0 "U18" H 9700 4350 60  0000 C CNN
F 1 "PowerDist_WiretoBoard" H 10150 4250 60  0000 C CNN
F 2 "zeabus:PowerDist_WiretoBoard" H 10150 4250 60  0001 C CNN
F 3 "" H 10150 4250 60  0001 C CNN
	1    10150 4250
	1    0    0    -1  
$EndComp
$Comp
L Pololu_SimpleMotorController24v23_withnoPowerDist U14
U 1 1 5A5AFE9F
P 8900 4450
F 0 "U14" H 8500 5350 60  0000 C CNN
F 1 "Pololu_SimpleMotorController24v23_withnoPowerDist" H 9250 5250 60  0000 C CNN
F 2 "zeabus:pololu_motorcontroller24v23" H 9250 5250 60  0001 C CNN
F 3 "" H 9250 5250 60  0001 C CNN
	1    8900 4450
	1    0    0    -1  
$EndComp
$Comp
L PowerDist_WiretoBoard U17
U 1 1 5A5AFEA5
P 10150 2750
F 0 "U17" H 9700 2850 60  0000 C CNN
F 1 "PowerDist_WiretoBoard" H 10150 2750 60  0000 C CNN
F 2 "zeabus:PowerDist_WiretoBoard" H 10150 2750 60  0001 C CNN
F 3 "" H 10150 2750 60  0001 C CNN
	1    10150 2750
	1    0    0    -1  
$EndComp
$Comp
L Pololu_SimpleMotorController24v23_withnoPowerDist U13
U 1 1 5A5AFEAB
P 8900 2950
F 0 "U13" H 8500 3850 60  0000 C CNN
F 1 "Pololu_SimpleMotorController24v23_withnoPowerDist" H 9250 3750 60  0000 C CNN
F 2 "zeabus:pololu_motorcontroller24v23" H 9250 3750 60  0001 C CNN
F 3 "" H 9250 3750 60  0001 C CNN
	1    8900 2950
	1    0    0    -1  
$EndComp
$Comp
L PowerDist_WiretoBoard U16
U 1 1 5A5AFEB1
P 10150 1250
F 0 "U16" H 9700 1350 60  0000 C CNN
F 1 "PowerDist_WiretoBoard" H 10150 1250 60  0000 C CNN
F 2 "zeabus:PowerDist_WiretoBoard" H 10150 1250 60  0001 C CNN
F 3 "" H 10150 1250 60  0001 C CNN
	1    10150 1250
	1    0    0    -1  
$EndComp
$Comp
L Pololu_SimpleMotorController24v23_withnoPowerDist U12
U 1 1 5A5AFEB7
P 8900 1450
F 0 "U12" H 8500 2350 60  0000 C CNN
F 1 "Pololu_SimpleMotorController24v23_withnoPowerDist" H 9250 2250 60  0000 C CNN
F 2 "zeabus:pololu_motorcontroller24v23" H 9250 2250 60  0001 C CNN
F 3 "" H 9250 2250 60  0001 C CNN
	1    8900 1450
	1    0    0    -1  
$EndComp
NoConn ~ 5250 4000
NoConn ~ 5250 5500
NoConn ~ 8250 5500
NoConn ~ 8250 4000
NoConn ~ 8250 1000
NoConn ~ 5350 5100
Text Label 4950 1200 0    60   ~ 0
signal0
Text Label 4950 2700 0    60   ~ 0
signal1
Text Label 4950 4200 0    60   ~ 0
signal2
Text Label 4950 5700 0    60   ~ 0
signal3
Text Label 8000 1200 0    60   ~ 0
signal6
Text Label 7950 2700 0    60   ~ 0
signal7
Text Label 7950 4200 0    60   ~ 0
signal8
Text Label 7950 5700 0    60   ~ 0
signal9
$Comp
L GNDA #PWR010
U 1 1 5A5B4DC7
P 5250 1850
F 0 "#PWR010" H 5250 1600 50  0001 C CNN
F 1 "GNDA" H 5250 1700 50  0000 C CNN
F 2 "" H 5250 1850 50  0001 C CNN
F 3 "" H 5250 1850 50  0001 C CNN
	1    5250 1850
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR011
U 1 1 5A5B4E29
P 5250 3350
F 0 "#PWR011" H 5250 3100 50  0001 C CNN
F 1 "GNDA" H 5250 3200 50  0000 C CNN
F 2 "" H 5250 3350 50  0001 C CNN
F 3 "" H 5250 3350 50  0001 C CNN
	1    5250 3350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR012
U 1 1 5A5B4E8B
P 5250 4850
F 0 "#PWR012" H 5250 4600 50  0001 C CNN
F 1 "GNDA" H 5250 4700 50  0000 C CNN
F 2 "" H 5250 4850 50  0001 C CNN
F 3 "" H 5250 4850 50  0001 C CNN
	1    5250 4850
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR013
U 1 1 5A5B4EED
P 5250 6350
F 0 "#PWR013" H 5250 6100 50  0001 C CNN
F 1 "GNDA" H 5250 6200 50  0000 C CNN
F 2 "" H 5250 6350 50  0001 C CNN
F 3 "" H 5250 6350 50  0001 C CNN
	1    5250 6350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR014
U 1 1 5A5B5241
P 8250 6350
F 0 "#PWR014" H 8250 6100 50  0001 C CNN
F 1 "GNDA" H 8250 6200 50  0000 C CNN
F 2 "" H 8250 6350 50  0001 C CNN
F 3 "" H 8250 6350 50  0001 C CNN
	1    8250 6350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR015
U 1 1 5A5B52A3
P 8250 4850
F 0 "#PWR015" H 8250 4600 50  0001 C CNN
F 1 "GNDA" H 8250 4700 50  0000 C CNN
F 2 "" H 8250 4850 50  0001 C CNN
F 3 "" H 8250 4850 50  0001 C CNN
	1    8250 4850
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR016
U 1 1 5A5B5366
P 8250 3350
F 0 "#PWR016" H 8250 3100 50  0001 C CNN
F 1 "GNDA" H 8250 3200 50  0000 C CNN
F 2 "" H 8250 3350 50  0001 C CNN
F 3 "" H 8250 3350 50  0001 C CNN
	1    8250 3350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR017
U 1 1 5A5B598F
P 8250 1850
F 0 "#PWR017" H 8250 1600 50  0001 C CNN
F 1 "GNDA" H 8250 1700 50  0000 C CNN
F 2 "" H 8250 1850 50  0001 C CNN
F 3 "" H 8250 1850 50  0001 C CNN
	1    8250 1850
	1    0    0    -1  
$EndComp
Text Label 6450 1500 0    60   ~ 0
BATTIN_0
Text Label 6450 2950 0    60   ~ 0
BATTIN_0
Text Label 6450 4500 0    60   ~ 0
BATTIN_0
Text Label 6450 6000 0    60   ~ 0
BATTIN_0
Text Label 6350 1700 0    60   ~ 0
BATTGND
Text Label 6350 3150 0    60   ~ 0
BATTGND
Text Label 6350 4700 0    60   ~ 0
BATTGND
Text Label 6350 6200 0    60   ~ 0
BATTGND
Text Label 9450 1500 0    60   ~ 0
BATTIN_1
Text Label 9450 3000 0    60   ~ 0
BATTIN_1
Text Label 9450 4500 0    60   ~ 0
BATTIN_1
Text Label 9450 6000 0    60   ~ 0
BATTIN_1
Text Label 9350 1700 0    60   ~ 0
BATTGND
Text Label 9350 3200 0    60   ~ 0
BATTGND
Text Label 9350 4700 0    60   ~ 0
BATTGND
Text Label 9350 6200 0    60   ~ 0
BATTGND
Text Label 7650 1500 0    60   ~ 0
OUTA_0
Text Label 7650 1700 0    60   ~ 0
OUTB_0
Text Label 10650 1500 0    60   ~ 0
OUTA_4
Text Label 10650 1700 0    60   ~ 0
OUTB_4
Text Label 7650 2950 0    60   ~ 0
OUTA_1
Text Label 7650 3150 0    60   ~ 0
OUTB_1
Text Label 10650 3000 0    60   ~ 0
OUTA_5
Text Label 10650 3200 0    60   ~ 0
OUTB_5
Text Label 7650 4500 0    60   ~ 0
OUTA_2
Text Label 7650 4700 0    60   ~ 0
OUTB_2
Text Label 10650 4500 0    60   ~ 0
OUTA_6
Text Label 10650 4700 0    60   ~ 0
OUTB_6
Text Label 7650 6000 0    60   ~ 0
OUTA_3
Text Label 7650 6200 0    60   ~ 0
OUTB_3
Text Label 10650 6000 0    60   ~ 0
OUTA_7
Text Label 10650 6200 0    60   ~ 0
OUTB_7
$Comp
L Pololu_Maestro12 U1
U 1 1 5A5C0A4F
P 1900 3550
F 0 "U1" H 1600 3600 60  0000 C CNN
F 1 "Pololu_Maestro12" H 1900 3500 60  0000 C CNN
F 2 "zeabus:Pololu_Maestro12" H 1900 3500 60  0001 C CNN
F 3 "" H 1900 3500 60  0001 C CNN
	1    1900 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 3900 750  3900
Wire Wire Line
	750  3900 750  3050
Wire Wire Line
	1300 4650 750  4650
Wire Wire Line
	750  4650 750  5550
Wire Wire Line
	1300 4750 1050 4750
Wire Wire Line
	1050 4750 1050 5550
Wire Wire Line
	2500 3700 2500 2700
Wire Wire Line
	2500 3800 2600 3800
Wire Wire Line
	2600 3800 2600 2800
Wire Wire Line
	2500 3900 2700 3900
Wire Wire Line
	2700 3900 2700 2900
Wire Wire Line
	2500 4000 2800 4000
Wire Wire Line
	2800 4000 2800 3000
Wire Wire Line
	2500 4100 2900 4100
Wire Wire Line
	2900 4100 2900 3100
Wire Wire Line
	2500 4200 3000 4200
Wire Wire Line
	3000 4200 3000 3200
Wire Wire Line
	2500 4700 2600 4700
Wire Wire Line
	2600 4700 2600 5700
Wire Wire Line
	2500 4600 2700 4600
Wire Wire Line
	2700 4600 2700 5600
Wire Wire Line
	2500 4500 2800 4500
Wire Wire Line
	2800 4500 2800 5500
Wire Wire Line
	2500 5800 2500 4800
Wire Wire Line
	2900 4400 2900 5400
Wire Wire Line
	2500 4400 2900 4400
Wire Wire Line
	2500 4300 3000 4300
Wire Wire Line
	3000 4300 3000 5300
Wire Wire Line
	900  750  1200 750 
Wire Wire Line
	900  850  1200 850 
Wire Wire Line
	900  1200 1200 1200
Wire Wire Line
	900  1300 1200 1300
Wire Wire Line
	1600 750  1900 750 
Wire Wire Line
	1600 850  1900 850 
Wire Wire Line
	1600 1200 1900 1200
Wire Wire Line
	1600 1300 1900 1300
Wire Wire Line
	2300 750  2600 750 
Wire Wire Line
	2300 850  2600 850 
Wire Wire Line
	2300 1200 2600 1200
Wire Wire Line
	2300 1300 2600 1300
Wire Wire Line
	3000 1300 3300 1300
Wire Wire Line
	3000 1200 3300 1200
Wire Wire Line
	3000 850  3300 850 
Wire Wire Line
	3000 750  3300 750 
Wire Wire Line
	1850 7000 2250 7000
Wire Wire Line
	1750 7100 2250 7100
Wire Wire Line
	2750 7000 3150 7000
Wire Wire Line
	2650 7100 3150 7100
Wire Wire Line
	3000 3200 3250 3200
Wire Wire Line
	2900 3100 3250 3100
Wire Wire Line
	2800 3000 3250 3000
Wire Wire Line
	2700 2900 3250 2900
Wire Wire Line
	2600 2800 3250 2800
Wire Wire Line
	2500 2700 3250 2700
Wire Wire Line
	3000 5300 3250 5300
Wire Wire Line
	2900 5400 3250 5400
Wire Wire Line
	2800 5500 3250 5500
Wire Wire Line
	2700 5600 3250 5600
Wire Wire Line
	2600 5700 3250 5700
Wire Wire Line
	3250 5800 2500 5800
Wire Wire Line
	4950 1200 5250 1200
Wire Wire Line
	4950 2700 5250 2700
Wire Wire Line
	4950 4200 5250 4200
Wire Wire Line
	4950 5700 5250 5700
Wire Wire Line
	7950 5700 8250 5700
Wire Wire Line
	7950 4200 8250 4200
Wire Wire Line
	7950 2700 8250 2700
Wire Wire Line
	8000 1200 8250 1200
Wire Wire Line
	6450 1500 6800 1500
Wire Wire Line
	6800 1700 6350 1700
Wire Wire Line
	6450 2950 6800 2950
Wire Wire Line
	6800 3150 6350 3150
Wire Wire Line
	6450 4300 6800 4300
Wire Wire Line
	6800 4500 6350 4500
Wire Wire Line
	6450 5800 6800 5800
Wire Wire Line
	6800 6000 6350 6000
Wire Wire Line
	9350 6200 9800 6200
Wire Wire Line
	9800 6000 9450 6000
Wire Wire Line
	9350 4700 9800 4700
Wire Wire Line
	9800 4500 9450 4500
Wire Wire Line
	9800 3200 9350 3200
Wire Wire Line
	9450 3000 9800 3000
Wire Wire Line
	9350 1700 9800 1700
Wire Wire Line
	9450 1500 9800 1500
Wire Wire Line
	6350 4700 6800 4700
Wire Wire Line
	6800 6200 6350 6200
Text Label 4250 2500 0    60   ~ 0
+5v_BEC
Text Label 4250 5100 0    60   ~ 0
+5v_BEC
Text Label 7900 2500 0    60   ~ 0
+5v_BEC
Wire Wire Line
	7900 2500 8250 2500
NoConn ~ 5250 2500
$EndSCHEMATC
