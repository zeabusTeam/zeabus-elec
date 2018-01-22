EESchema Schematic File Version 2
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
LIBS:Backplane-cache
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
L Conn-PCIE-x1 CON?
U 1 1 5A5641E3
P 7500 4200
F 0 "CON?" H 7650 4250 60  0000 C CNN
F 1 "peripheral_bridge" H 7900 2100 60  0000 C CNN
F 2 "Zeabus:pcie-x1-conn" H 7500 4200 60  0001 C CNN
F 3 "" H 7500 4200 60  0001 C CNN
	1    7500 4200
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A5644F5
P 1750 6050
F 0 "R?" V 1830 6050 50  0000 C CNN
F 1 "4.7K" V 1750 6050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1680 6050 50  0001 C CNN
F 3 "" H 1750 6050 50  0001 C CNN
	1    1750 6050
	0    -1   -1   0   
$EndComp
$Comp
L USB_OTG J?
U 1 1 5A564864
P 850 3900
F 0 "J?" H 650 4350 50  0000 L CNN
F 1 "USB_OTG" H 650 4250 50  0000 L CNN
F 2 "Connectors:USB_Mini-B" H 1000 3850 50  0001 C CNN
F 3 "" H 1000 3850 50  0001 C CNN
	1    850  3900
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A565759
P 1100 4500
F 0 "C?" H 1125 4600 50  0000 L CNN
F 1 "0.1uF" H 1125 4400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1138 4350 50  0001 C CNN
F 3 "" H 1100 4500 50  0001 C CNN
	1    1100 4500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A5657D2
P 1400 4500
F 0 "C?" H 1425 4600 50  0000 L CNN
F 1 "0.1uF" H 1425 4400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1438 4350 50  0001 C CNN
F 3 "" H 1400 4500 50  0001 C CNN
	1    1400 4500
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A565988
P 1700 4500
F 0 "R?" V 1780 4500 50  0000 C CNN
F 1 "1M" V 1700 4500 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1630 4500 50  0001 C CNN
F 3 "" H 1700 4500 50  0001 C CNN
	1    1700 4500
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR?
U 1 1 5A565F4A
P 750 4350
F 0 "#PWR?" H 750 4150 50  0001 C CNN
F 1 "GNDPWR" H 750 4220 50  0000 C CNN
F 2 "" H 750 4300 50  0001 C CNN
F 3 "" H 750 4300 50  0001 C CNN
	1    750  4350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A565F9E
P 1400 4650
F 0 "#PWR?" H 1400 4400 50  0001 C CNN
F 1 "GND" H 1400 4500 50  0000 C CNN
F 2 "" H 1400 4650 50  0001 C CNN
F 3 "" H 1400 4650 50  0001 C CNN
	1    1400 4650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A5670AE
P 2400 1850
F 0 "C?" H 2425 1950 50  0000 L CNN
F 1 "0.1uF" H 2425 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2438 1700 50  0001 C CNN
F 3 "" H 2400 1850 50  0001 C CNN
	1    2400 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A567149
P 2650 1850
F 0 "C?" H 2675 1950 50  0000 L CNN
F 1 "0.1uF" H 2675 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2688 1700 50  0001 C CNN
F 3 "" H 2650 1850 50  0001 C CNN
	1    2650 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A56719D
P 2900 1850
F 0 "C?" H 2925 1950 50  0000 L CNN
F 1 "0.1uF" H 2925 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2938 1700 50  0001 C CNN
F 3 "" H 2900 1850 50  0001 C CNN
	1    2900 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A5671A3
P 3150 1850
F 0 "C?" H 3175 1950 50  0000 L CNN
F 1 "0.1uF" H 3175 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3188 1700 50  0001 C CNN
F 3 "" H 3150 1850 50  0001 C CNN
	1    3150 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A5671EC
P 3400 1850
F 0 "C?" H 3425 1950 50  0000 L CNN
F 1 "0.1uF" H 3425 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3438 1700 50  0001 C CNN
F 3 "" H 3400 1850 50  0001 C CNN
	1    3400 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A56779F
P 5000 1850
F 0 "C?" H 5025 1950 50  0000 L CNN
F 1 "0.1uF" H 5025 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5038 1700 50  0001 C CNN
F 3 "" H 5000 1850 50  0001 C CNN
	1    5000 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A567933
P 4750 1850
F 0 "C?" H 4775 1950 50  0000 L CNN
F 1 "0.1uF" H 4775 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4788 1700 50  0001 C CNN
F 3 "" H 4750 1850 50  0001 C CNN
	1    4750 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A567C96
P 5500 1850
F 0 "C?" H 5525 1950 50  0000 L CNN
F 1 "0.1uF" H 5525 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5538 1700 50  0001 C CNN
F 3 "" H 5500 1850 50  0001 C CNN
	1    5500 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A567C9C
P 5250 1850
F 0 "C?" H 5275 1950 50  0000 L CNN
F 1 "0.1uF" H 5275 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5288 1700 50  0001 C CNN
F 3 "" H 5250 1850 50  0001 C CNN
	1    5250 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A567D38
P 6000 1850
F 0 "C?" H 6025 1950 50  0000 L CNN
F 1 "0.1uF" H 6025 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6038 1700 50  0001 C CNN
F 3 "" H 6000 1850 50  0001 C CNN
	1    6000 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A567D3E
P 5750 1850
F 0 "C?" H 5775 1950 50  0000 L CNN
F 1 "0.1uF" H 5775 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5788 1700 50  0001 C CNN
F 3 "" H 5750 1850 50  0001 C CNN
	1    5750 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A567D44
P 6500 1850
F 0 "C?" H 6525 1950 50  0000 L CNN
F 1 "10uF" H 6525 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6538 1700 50  0001 C CNN
F 3 "" H 6500 1850 50  0001 C CNN
	1    6500 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A567D4A
P 6250 1850
F 0 "C?" H 6275 1950 50  0000 L CNN
F 1 "0.1uF" H 6275 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6288 1700 50  0001 C CNN
F 3 "" H 6250 1850 50  0001 C CNN
	1    6250 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A569262
P 2900 2000
F 0 "#PWR?" H 2900 1750 50  0001 C CNN
F 1 "GND" H 2900 1850 50  0000 C CNN
F 2 "" H 2900 2000 50  0001 C CNN
F 3 "" H 2900 2000 50  0001 C CNN
	1    2900 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A56942A
P 5750 2000
F 0 "#PWR?" H 5750 1750 50  0001 C CNN
F 1 "GND" H 5750 1850 50  0000 C CNN
F 2 "" H 5750 2000 50  0001 C CNN
F 3 "" H 5750 2000 50  0001 C CNN
	1    5750 2000
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A569B61
P 2000 2800
F 0 "R?" V 2080 2800 50  0000 C CNN
F 1 "10K1%" V 2000 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1930 2800 50  0001 C CNN
F 3 "" H 2000 2800 50  0001 C CNN
	1    2000 2800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A56A0EA
P 2000 2950
F 0 "#PWR?" H 2000 2700 50  0001 C CNN
F 1 "GND" H 2000 2800 50  0000 C CNN
F 2 "" H 2000 2950 50  0001 C CNN
F 3 "" H 2000 2950 50  0001 C CNN
	1    2000 2950
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A56A273
P 1700 2650
F 0 "R?" V 1780 2650 50  0000 C CNN
F 1 "90.9K" V 1700 2650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1630 2650 50  0001 C CNN
F 3 "" H 1700 2650 50  0001 C CNN
	1    1700 2650
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5A56A392
P 1400 2800
F 0 "C?" H 1425 2900 50  0000 L CNN
F 1 "10uF" H 1425 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1438 2650 50  0001 C CNN
F 3 "" H 1400 2800 50  0001 C CNN
	1    1400 2800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A56A5B7
P 1400 2950
F 0 "#PWR?" H 1400 2700 50  0001 C CNN
F 1 "GND" H 1400 2800 50  0000 C CNN
F 2 "" H 1400 2950 50  0001 C CNN
F 3 "" H 1400 2950 50  0001 C CNN
	1    1400 2950
	1    0    0    -1  
$EndComp
$Comp
L TLV733 U?
U 1 1 5A56AC20
P 5250 850
F 0 "U?" H 5350 900 60  0000 C CNN
F 1 "TLV73311PDBVT" H 5400 450 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5_HandSoldering" H 5250 900 60  0001 C CNN
F 3 "" H 5250 900 60  0001 C CNN
	1    5250 850 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A56B0DD
P 4850 1100
F 0 "C?" H 4875 1200 50  0000 L CNN
F 1 "1uF" H 4875 1000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4888 950 50  0001 C CNN
F 3 "" H 4850 1100 50  0001 C CNN
	1    4850 1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A56B422
P 4850 1250
F 0 "#PWR?" H 4850 1000 50  0001 C CNN
F 1 "GND" H 4850 1100 50  0000 C CNN
F 2 "" H 4850 1250 50  0001 C CNN
F 3 "" H 4850 1250 50  0001 C CNN
	1    4850 1250
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A56B583
P 6300 1100
F 0 "C?" H 6325 1200 50  0000 L CNN
F 1 "1uF" H 6325 1000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6338 950 50  0001 C CNN
F 3 "" H 6300 1100 50  0001 C CNN
	1    6300 1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A56B838
P 6300 1250
F 0 "#PWR?" H 6300 1000 50  0001 C CNN
F 1 "GND" H 6300 1100 50  0000 C CNN
F 2 "" H 6300 1250 50  0001 C CNN
F 3 "" H 6300 1250 50  0001 C CNN
	1    6300 1250
	1    0    0    -1  
$EndComp
$Comp
L CP1 C?
U 1 1 5A56B9D4
P 1650 1150
F 0 "C?" H 1675 1250 50  0000 L CNN
F 1 "10uF" H 1675 1050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1650 1150 50  0001 C CNN
F 3 "" H 1650 1150 50  0001 C CNN
	1    1650 1150
	1    0    0    -1  
$EndComp
$Comp
L CP1 C?
U 1 1 5A56BC11
P 3200 1150
F 0 "C?" H 3225 1250 50  0000 L CNN
F 1 "10uF" H 3225 1050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3200 1150 50  0001 C CNN
F 3 "" H 3200 1150 50  0001 C CNN
	1    3200 1150
	1    0    0    -1  
$EndComp
$Comp
L LM1117-XX U?
U 1 1 5A56BD0C
P 2450 1050
F 0 "U?" H 2250 1250 40  0000 C CNN
F 1 "LM1117-3.3" H 2450 1250 40  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 2450 1150 30  0000 C CIN
F 3 "" H 2250 1250 60  0000 C CNN
	1    2450 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A56C69A
P 2450 1300
F 0 "#PWR?" H 2450 1050 50  0001 C CNN
F 1 "GND" H 2450 1150 50  0000 C CNN
F 2 "" H 2450 1300 50  0001 C CNN
F 3 "" H 2450 1300 50  0001 C CNN
	1    2450 1300
	1    0    0    -1  
$EndComp
Text GLabel 1700 3700 2    60   Input ~ 0
VBUS
$Comp
L R R?
U 1 1 5A571986
P 3100 4200
F 0 "R?" V 3180 4200 50  0000 C CNN
F 1 "9.53K" V 3100 4200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3030 4200 50  0001 C CNN
F 3 "" H 3100 4200 50  0001 C CNN
	1    3100 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A571B4C
P 3100 4350
F 0 "#PWR?" H 3100 4100 50  0001 C CNN
F 1 "GND" H 3100 4200 50  0000 C CNN
F 2 "" H 3100 4350 50  0001 C CNN
F 3 "" H 3100 4350 50  0001 C CNN
	1    3100 4350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A57A9A0
P 9200 1800
F 0 "C?" H 9225 1900 50  0000 L CNN
F 1 "0.1uF" H 9225 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 9238 1650 50  0001 C CNN
F 3 "" H 9200 1800 50  0001 C CNN
	1    9200 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A57B23E
P 9200 1950
F 0 "#PWR?" H 9200 1700 50  0001 C CNN
F 1 "GND" H 9200 1800 50  0000 C CNN
F 2 "" H 9200 1950 50  0001 C CNN
F 3 "" H 9200 1950 50  0001 C CNN
	1    9200 1950
	1    0    0    -1  
$EndComp
Text GLabel 9500 1450 0    60   Input ~ 0
VPCIE
Text GLabel 11100 1600 2    60   Input ~ 0
VPCIE
$Comp
L C C?
U 1 1 5A57E400
P 6300 5000
F 0 "C?" H 6325 5100 50  0000 L CNN
F 1 "0.1uF" H 6325 4900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6338 4850 50  0001 C CNN
F 3 "" H 6300 5000 50  0001 C CNN
	1    6300 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A57E709
P 6300 5150
F 0 "#PWR?" H 6300 4900 50  0001 C CNN
F 1 "GND" H 6300 5000 50  0000 C CNN
F 2 "" H 6300 5150 50  0001 C CNN
F 3 "" H 6300 5150 50  0001 C CNN
	1    6300 5150
	1    0    0    -1  
$EndComp
$Comp
L XT30 P?
U 1 1 5A581965
P 4600 7400
F 0 "P?" H 4600 7550 50  0000 C CNN
F 1 "ethernet_switch" H 4600 7250 50  0000 C CNN
F 2 "Zeabus:XT30" H 4600 7400 60  0000 C CNN
F 3 "" H 4600 7400 60  0000 C CNN
	1    4600 7400
	1    0    0    -1  
$EndComp
$Comp
L XT30 P?
U 1 1 5A581A2E
P 2150 7400
F 0 "P?" H 2150 7550 50  0000 C CNN
F 1 "DVL" H 2150 7250 50  0000 C CNN
F 2 "Zeabus:XT30" H 2150 7400 60  0000 C CNN
F 3 "" H 2150 7400 60  0000 C CNN
	1    2150 7400
	1    0    0    -1  
$EndComp
$Comp
L XT30 P?
U 1 1 5A581BC8
P 2950 7400
F 0 "P?" H 2950 7550 50  0000 C CNN
F 1 "computer1" H 2950 7250 50  0000 C CNN
F 2 "Zeabus:XT30" H 2950 7400 60  0000 C CNN
F 3 "" H 2950 7400 60  0000 C CNN
	1    2950 7400
	1    0    0    -1  
$EndComp
$Comp
L XT30 P?
U 1 1 5A581C5D
P 3800 7400
F 0 "P?" H 3800 7550 50  0000 C CNN
F 1 "computer2" H 3800 7250 50  0000 C CNN
F 2 "Zeabus:XT30" H 3800 7400 60  0000 C CNN
F 3 "" H 3800 7400 60  0000 C CNN
	1    3800 7400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A58309D
P 4300 7500
F 0 "#PWR?" H 4300 7250 50  0001 C CNN
F 1 "GND" H 4300 7350 50  0000 C CNN
F 2 "" H 4300 7500 50  0001 C CNN
F 3 "" H 4300 7500 50  0001 C CNN
	1    4300 7500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A583246
P 2650 7500
F 0 "#PWR?" H 2650 7250 50  0001 C CNN
F 1 "GND" H 2650 7350 50  0000 C CNN
F 2 "" H 2650 7500 50  0001 C CNN
F 3 "" H 2650 7500 50  0001 C CNN
	1    2650 7500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A5832C8
P 3500 7500
F 0 "#PWR?" H 3500 7250 50  0001 C CNN
F 1 "GND" H 3500 7350 50  0000 C CNN
F 2 "" H 3500 7500 50  0001 C CNN
F 3 "" H 3500 7500 50  0001 C CNN
	1    3500 7500
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J?
U 1 1 5A584C3E
P 9700 4700
F 0 "J?" H 9700 4900 50  0000 C CNN
F 1 "RS232_DVL" H 9700 4500 50  0000 C CNN
F 2 "Zeabus:90136-2013" H 9700 4700 50  0001 C CNN
F 3 "" H 9700 4700 50  0001 C CNN
	1    9700 4700
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J?
U 1 1 5A5725F7
P 9700 4250
F 0 "J?" H 9700 4450 50  0000 C CNN
F 1 "pressure_sensor" H 9700 4050 50  0000 C CNN
F 2 "Zeabus:90136-2013" H 9700 4250 50  0001 C CNN
F 3 "" H 9700 4250 50  0001 C CNN
	1    9700 4250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A5CC94C
P 8500 6250
F 0 "#PWR?" H 8500 6000 50  0001 C CNN
F 1 "GND" H 8500 6100 50  0000 C CNN
F 2 "" H 8500 6250 50  0001 C CNN
F 3 "" H 8500 6250 50  0001 C CNN
	1    8500 6250
	1    0    0    -1  
$EndComp
$Comp
L Ferrite_Bead L?
U 1 1 5A5CEB74
P 5900 4850
F 0 "L?" V 5750 4875 50  0000 C CNN
F 1 "220 @ 100Mhz" V 6050 4850 50  0000 C CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" V 5830 4850 50  0001 C CNN
F 3 "" H 5900 4850 50  0001 C CNN
	1    5900 4850
	0    1    1    0   
$EndComp
$Comp
L Ferrite_Bead L?
U 1 1 5A5CF6BA
P 8800 1650
F 0 "L?" V 8650 1675 50  0000 C CNN
F 1 "220 @ 100Mhz" V 8950 1650 50  0000 C CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" V 8730 1650 50  0001 C CNN
F 3 "" H 8800 1650 50  0001 C CNN
	1    8800 1650
	0    1    1    0   
$EndComp
$Comp
L Conn_02x04_Counter_Clockwise J?
U 1 1 5A5D0A10
P 9650 5350
F 0 "J?" H 9700 5550 50  0000 C CNN
F 1 "pneumatic" H 9700 5050 50  0000 C CNN
F 2 "Zeabus:Pin_Header_Straight_2x04" H 9650 5350 50  0001 C CNN
F 3 "" H 9650 5350 50  0001 C CNN
	1    9650 5350
	1    0    0    -1  
$EndComp
Text GLabel 1600 2450 2    60   Input ~ 0
VBUS
$Comp
L Conn_Jumper J?
U 1 1 5A5DEBD0
P 1050 5700
F 0 "J?" H 1050 5900 50  0000 C CNN
F 1 "power_source_mode" H 1050 5500 50  0000 C CNN
F 2 "Zeabus:90136-2013" H 1100 5700 50  0001 C CNN
F 3 "" H 1100 5700 50  0001 C CNN
	1    1050 5700
	1    0    0    -1  
$EndComp
Text GLabel 850  5600 0    60   Input ~ 0
VBUS
Text GLabel 850  5800 0    60   Input ~ 0
VPCIE
$Comp
L +3.3V #PWR?
U 1 1 5A60DFC7
P 1600 6050
F 0 "#PWR?" H 1600 5900 50  0001 C CNN
F 1 "+3.3V" H 1600 6190 50  0000 C CNN
F 2 "" H 1600 6050 50  0001 C CNN
F 3 "" H 1600 6050 50  0001 C CNN
	1    1600 6050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A60CD30
P 3200 1000
F 0 "#PWR?" H 3200 850 50  0001 C CNN
F 1 "+3.3V" H 3200 1140 50  0000 C CNN
F 2 "" H 3200 1000 50  0001 C CNN
F 3 "" H 3200 1000 50  0001 C CNN
	1    3200 1000
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A6127D1
P 1750 5850
F 0 "R?" V 1830 5850 50  0000 C CNN
F 1 "4.7K" V 1750 5850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1680 5850 50  0001 C CNN
F 3 "" H 1750 5850 50  0001 C CNN
	1    1750 5850
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A614A5C
P 1600 5850
F 0 "#PWR?" H 1600 5700 50  0001 C CNN
F 1 "+3.3V" H 1600 5990 50  0000 C CNN
F 2 "" H 1600 5850 50  0001 C CNN
F 3 "" H 1600 5850 50  0001 C CNN
	1    1600 5850
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A6197DD
P 2850 6250
F 0 "R?" V 2930 6250 50  0000 C CNN
F 1 "4.7K" V 2850 6250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2780 6250 50  0001 C CNN
F 3 "" H 2850 6250 50  0001 C CNN
	1    2850 6250
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5A619BE9
P 3500 6550
F 0 "#PWR?" H 3500 6300 50  0001 C CNN
F 1 "GND" H 3500 6400 50  0000 C CNN
F 2 "" H 3500 6550 50  0001 C CNN
F 3 "" H 3500 6550 50  0001 C CNN
	1    3500 6550
	1    0    0    -1  
$EndComp
$Comp
L TUSB4041I U?
U 1 1 5A564068
P 4400 4450
F 0 "U?" H 5000 6550 50  0000 L CNN
F 1 "TUSB4041I" H 5000 6450 50  0000 L CNN
F 2 "Housings_QFP:HTQFP-64_1EP_10x10mm_Pitch0.5mm_ThermalPad" H 5600 6450 50  0001 L CNN
F 3 "" H 4100 4650 50  0001 C CNN
	1    4400 4450
	1    0    0    -1  
$EndComp
Text GLabel 4250 7350 0    60   Input ~ 0
VPCIE
$Comp
L +12V #PWR?
U 1 1 5A60F4A5
P 2450 7350
F 0 "#PWR?" H 2450 7200 50  0001 C CNN
F 1 "+12V" H 2450 7490 50  0000 C CNN
F 2 "" H 2450 7350 50  0001 C CNN
F 3 "" H 2450 7350 50  0001 C CNN
	1    2450 7350
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 5A60F51D
P 3300 7350
F 0 "#PWR?" H 3300 7200 50  0001 C CNN
F 1 "+12V" H 3300 7490 50  0000 C CNN
F 2 "" H 3300 7350 50  0001 C CNN
F 3 "" H 3300 7350 50  0001 C CNN
	1    3300 7350
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 5A60FB3B
P 8550 4450
F 0 "#PWR?" H 8550 4300 50  0001 C CNN
F 1 "+12V" H 8550 4590 50  0000 C CNN
F 2 "" H 8550 4450 50  0001 C CNN
F 3 "" H 8550 4450 50  0001 C CNN
	1    8550 4450
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 5A60FBB3
P 7200 4400
F 0 "#PWR?" H 7200 4250 50  0001 C CNN
F 1 "+12V" H 7200 4540 50  0000 C CNN
F 2 "" H 7200 4400 50  0001 C CNN
F 3 "" H 7200 4400 50  0001 C CNN
	1    7200 4400
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5A60F609
P 1250 5700
F 0 "#PWR?" H 1250 5550 50  0001 C CNN
F 1 "+5V" H 1250 5840 50  0000 C CNN
F 2 "" H 1250 5700 50  0001 C CNN
F 3 "" H 1250 5700 50  0001 C CNN
	1    1250 5700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5A60FB92
P 1650 1000
F 0 "#PWR?" H 1650 850 50  0001 C CNN
F 1 "+5V" H 1650 1140 50  0000 C CNN
F 2 "" H 1650 1000 50  0001 C CNN
F 3 "" H 1650 1000 50  0001 C CNN
	1    1650 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5A610464
P 4850 950
F 0 "#PWR?" H 4850 800 50  0001 C CNN
F 1 "+5V" H 4850 1090 50  0000 C CNN
F 2 "" H 4850 950 50  0001 C CNN
F 3 "" H 4850 950 50  0001 C CNN
	1    4850 950 
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5A610DAA
P 5750 4850
F 0 "#PWR?" H 5750 4700 50  0001 C CNN
F 1 "+5V" H 5750 4990 50  0000 C CNN
F 2 "" H 5750 4850 50  0001 C CNN
F 3 "" H 5750 4850 50  0001 C CNN
	1    5750 4850
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5A611048
P 8650 1650
F 0 "#PWR?" H 8650 1500 50  0001 C CNN
F 1 "+5V" H 8650 1790 50  0000 C CNN
F 2 "" H 8650 1650 50  0001 C CNN
F 3 "" H 8650 1650 50  0001 C CNN
	1    8650 1650
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5A612588
P 8800 3800
F 0 "#PWR?" H 8800 3650 50  0001 C CNN
F 1 "+5V" H 8800 3940 50  0000 C CNN
F 2 "" H 8800 3800 50  0001 C CNN
F 3 "" H 8800 3800 50  0001 C CNN
	1    8800 3800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A61F5A4
P 1750 5550
F 0 "R?" V 1830 5550 50  0000 C CNN
F 1 "4.7K" V 1750 5550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1680 5550 50  0001 C CNN
F 3 "" H 1750 5550 50  0001 C CNN
	1    1750 5550
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A61F5AA
P 1600 5550
F 0 "#PWR?" H 1600 5400 50  0001 C CNN
F 1 "+3.3V" H 1600 5690 50  0000 C CNN
F 2 "" H 1600 5550 50  0001 C CNN
F 3 "" H 1600 5550 50  0001 C CNN
	1    1600 5550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A622BF5
P 6600 3150
F 0 "#PWR?" H 6600 2900 50  0001 C CNN
F 1 "GND" H 6600 3000 50  0000 C CNN
F 2 "" H 6600 3150 50  0001 C CNN
F 3 "" H 6600 3150 50  0001 C CNN
	1    6600 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A62378C
P 6600 3650
F 0 "#PWR?" H 6600 3400 50  0001 C CNN
F 1 "GND" H 6600 3500 50  0000 C CNN
F 2 "" H 6600 3650 50  0001 C CNN
F 3 "" H 6600 3650 50  0001 C CNN
	1    6600 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A623876
P 6600 4150
F 0 "#PWR?" H 6600 3900 50  0001 C CNN
F 1 "GND" H 6600 4000 50  0000 C CNN
F 2 "" H 6600 4150 50  0001 C CNN
F 3 "" H 6600 4150 50  0001 C CNN
	1    6600 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A623955
P 6600 4650
F 0 "#PWR?" H 6600 4400 50  0001 C CNN
F 1 "GND" H 6600 4500 50  0000 C CNN
F 2 "" H 6600 4650 50  0001 C CNN
F 3 "" H 6600 4650 50  0001 C CNN
	1    6600 4650
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A62395B
P 6200 4550
F 0 "R?" V 6280 4550 50  0000 C CNN
F 1 "4.7k" V 6200 4550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6130 4550 50  0001 C CNN
F 3 "" H 6200 4550 50  0001 C CNN
	1    6200 4550
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5A6242AD
P 2850 5950
F 0 "R?" V 2930 5950 50  0000 C CNN
F 1 "4.7K" V 2850 5950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2780 5950 50  0001 C CNN
F 3 "" H 2850 5950 50  0001 C CNN
	1    2850 5950
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5A64A969
P 2450 2900
F 0 "R?" V 2530 2900 50  0000 C CNN
F 1 "1M" V 2450 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2380 2900 50  0001 C CNN
F 3 "" H 2450 2900 50  0001 C CNN
	1    2450 2900
	0    1    1    0   
$EndComp
$Comp
L Crystal Y?
U 1 1 5A64AB53
P 2450 3200
F 0 "Y?" H 2450 3350 50  0000 C CNN
F 1 "24MHz" H 2450 3050 50  0000 C CNN
F 2 "Crystals:Crystal_SMD_Abracon_ABM3-2pin_5.0x3.2mm_HandSoldering" H 2450 3200 50  0001 C CNN
F 3 "" H 2450 3200 50  0001 C CNN
	1    2450 3200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A64AFC4
P 2300 3500
F 0 "C?" H 2325 3600 50  0000 L CNN
F 1 "18pF" H 2325 3400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2338 3350 50  0001 C CNN
F 3 "" H 2300 3500 50  0001 C CNN
	1    2300 3500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A64B075
P 2600 3500
F 0 "C?" H 2625 3600 50  0000 L CNN
F 1 "18pF" H 2625 3400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2638 3350 50  0001 C CNN
F 3 "" H 2600 3500 50  0001 C CNN
	1    2600 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A64B57C
P 2450 3700
F 0 "#PWR?" H 2450 3450 50  0001 C CNN
F 1 "GND" H 2450 3550 50  0000 C CNN
F 2 "" H 2450 3700 50  0001 C CNN
F 3 "" H 2450 3700 50  0001 C CNN
	1    2450 3700
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A64B82F
P 2900 3550
F 0 "C?" H 2925 3650 50  0000 L CNN
F 1 "1uF" H 2925 3450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2938 3400 50  0001 C CNN
F 3 "" H 2900 3550 50  0001 C CNN
	1    2900 3550
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5A6569A5
P 1550 7350
F 0 "#PWR?" H 1550 7200 50  0001 C CNN
F 1 "+BATT" H 1550 7490 50  0000 C CNN
F 2 "" H 1550 7350 50  0001 C CNN
F 3 "" H 1550 7350 50  0001 C CNN
	1    1550 7350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5A6573AE
P 1850 7500
F 0 "#PWR?" H 1850 7250 50  0001 C CNN
F 1 "GNDA" H 1850 7350 50  0000 C CNN
F 2 "" H 1850 7500 50  0001 C CNN
F 3 "" H 1850 7500 50  0001 C CNN
	1    1850 7500
	1    0    0    -1  
$EndComp
$Comp
L XT30 P?
U 1 1 5A6574CC
P 1300 7400
F 0 "P?" H 1300 7550 50  0000 C CNN
F 1 "imaging_sonar" H 1300 7250 50  0000 C CNN
F 2 "Zeabus:XT30" H 1300 7400 60  0000 C CNN
F 3 "" H 1300 7400 60  0000 C CNN
	1    1300 7400
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5A6574DE
P 700 7350
F 0 "#PWR?" H 700 7200 50  0001 C CNN
F 1 "+BATT" H 700 7490 50  0000 C CNN
F 2 "" H 700 7350 50  0001 C CNN
F 3 "" H 700 7350 50  0001 C CNN
	1    700  7350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5A6574E7
P 1000 7500
F 0 "#PWR?" H 1000 7250 50  0001 C CNN
F 1 "GNDA" H 1000 7350 50  0000 C CNN
F 2 "" H 1000 7500 50  0001 C CNN
F 3 "" H 1000 7500 50  0001 C CNN
	1    1000 7500
	1    0    0    -1  
$EndComp
$Comp
L Conn-PCIE-x16 CON?
U 1 1 5A66BAEE
P 10050 650
F 0 "CON?" H 10200 800 60  0000 C CNN
F 1 "power_distributor" H 10400 700 60  0000 C CNN
F 2 "Zeabus:pcie-x16-conn" H 10050 650 60  0001 C CNN
F 3 "" H 10050 650 60  0001 C CNN
	1    10050 650 
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 5A67CBEC
P 9750 800
F 0 "#PWR?" H 9750 650 50  0001 C CNN
F 1 "+12V" H 9750 940 50  0000 C CNN
F 2 "" H 9750 800 50  0001 C CNN
F 3 "" H 9750 800 50  0001 C CNN
	1    9750 800 
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 5A67D1BE
P 11050 650
F 0 "#PWR?" H 11050 500 50  0001 C CNN
F 1 "+12V" H 11050 790 50  0000 C CNN
F 2 "" H 11050 650 50  0001 C CNN
F 3 "" H 11050 650 50  0001 C CNN
	1    11050 650 
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5A67D3C9
P 11150 800
F 0 "#PWR?" H 11150 650 50  0001 C CNN
F 1 "+BATT" H 11150 940 50  0000 C CNN
F 2 "" H 11150 800 50  0001 C CNN
F 3 "" H 11150 800 50  0001 C CNN
	1    11150 800 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A67F851
P 11100 2600
F 0 "#PWR?" H 11100 2350 50  0001 C CNN
F 1 "GND" H 11100 2450 50  0000 C CNN
F 2 "" H 11100 2600 50  0001 C CNN
F 3 "" H 11100 2600 50  0001 C CNN
	1    11100 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6808E5
P 9750 2600
F 0 "#PWR?" H 9750 2350 50  0001 C CNN
F 1 "GND" H 9750 2450 50  0000 C CNN
F 2 "" H 9750 2600 50  0001 C CNN
F 3 "" H 9750 2600 50  0001 C CNN
	1    9750 2600
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5A680FAD
P 9600 1050
F 0 "#PWR?" H 9600 800 50  0001 C CNN
F 1 "GNDA" H 9600 900 50  0000 C CNN
F 2 "" H 9600 1050 50  0001 C CNN
F 3 "" H 9600 1050 50  0001 C CNN
	1    9600 1050
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A6923DC
P 9200 4400
F 0 "C?" H 9225 4500 50  0000 L CNN
F 1 "100nF" H 9225 4300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 9238 4250 50  0001 C CNN
F 3 "" H 9200 4400 50  0001 C CNN
	1    9200 4400
	1    0    0    -1  
$EndComp
$Comp
L L_Core_Iron L?
U 1 1 5A6930CD
P 9100 3800
F 0 "L?" V 9050 3800 50  0000 C CNN
F 1 "600R  0.5A" V 9210 3800 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9100 3800 50  0001 C CNN
F 3 "" H 9100 3800 50  0001 C CNN
	1    9100 3800
	0    -1   -1   0   
$EndComp
$Comp
L +12V #PWR?
U 1 1 5A696562
P 10150 5250
F 0 "#PWR?" H 10150 5100 50  0001 C CNN
F 1 "+12V" H 10150 5390 50  0000 C CNN
F 2 "" H 10150 5250 50  0001 C CNN
F 3 "" H 10150 5250 50  0001 C CNN
	1    10150 5250
	1    0    0    -1  
$EndComp
$Comp
L LED D?
U 1 1 5A628BE7
P 8200 650
F 0 "D?" H 8200 750 50  0000 C CNN
F 1 "LED_VPCIe" H 8200 550 50  0000 C CNN
F 2 "LEDs:LED_0603_HandSoldering" H 8200 650 50  0001 C CNN
F 3 "" H 8200 650 50  0001 C CNN
	1    8200 650 
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5A628E18
P 8500 650
F 0 "R?" V 8580 650 50  0000 C CNN
F 1 "1K" V 8500 650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8430 650 50  0001 C CNN
F 3 "" H 8500 650 50  0001 C CNN
	1    8500 650 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5A628F8E
P 8650 850
F 0 "#PWR?" H 8650 600 50  0001 C CNN
F 1 "GND" H 8650 700 50  0000 C CNN
F 2 "" H 8650 850 50  0001 C CNN
F 3 "" H 8650 850 50  0001 C CNN
	1    8650 850 
	1    0    0    -1  
$EndComp
Text GLabel 7000 650  0    60   Input ~ 0
VBUS
$Comp
L LED D?
U 1 1 5A63F044
P 7150 650
F 0 "D?" H 7150 750 50  0000 C CNN
F 1 "LED_VBUS" H 7150 550 50  0000 C CNN
F 2 "LEDs:LED_0603_HandSoldering" H 7150 650 50  0001 C CNN
F 3 "" H 7150 650 50  0001 C CNN
	1    7150 650 
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5A63F04A
P 7450 650
F 0 "R?" V 7530 650 50  0000 C CNN
F 1 "1K" V 7450 650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7380 650 50  0001 C CNN
F 3 "" H 7450 650 50  0001 C CNN
	1    7450 650 
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5A63F094
P 7600 850
F 0 "#PWR?" H 7600 600 50  0001 C CNN
F 1 "GND" H 7600 700 50  0000 C CNN
F 2 "" H 7600 850 50  0001 C CNN
F 3 "" H 7600 850 50  0001 C CNN
	1    7600 850 
	1    0    0    -1  
$EndComp
Text GLabel 8050 650  0    60   Input ~ 0
VPCIE
$Comp
L GND #PWR?
U 1 1 5A65DB26
P 5500 7500
F 0 "#PWR?" H 5500 7250 50  0001 C CNN
F 1 "GND" H 5500 7350 50  0000 C CNN
F 2 "" H 5500 7500 50  0001 C CNN
F 3 "" H 5500 7500 50  0001 C CNN
	1    5500 7500
	1    0    0    -1  
$EndComp
$Comp
L XT30 P?
U 1 1 5A65DB20
P 5800 7400
F 0 "P?" H 5800 7550 50  0000 C CNN
F 1 "general" H 5800 7250 50  0000 C CNN
F 2 "Zeabus:XT30" H 5800 7400 60  0000 C CNN
F 3 "" H 5800 7400 60  0000 C CNN
	1    5800 7400
	1    0    0    -1  
$EndComp
$Comp
L Conn_Jumper J?
U 1 1 5A65DF87
P 1050 6500
F 0 "J?" H 1050 6700 50  0000 C CNN
F 1 "general_power_source" H 1050 6300 50  0000 C CNN
F 2 "Zeabus:90136-2013" H 1100 6500 50  0001 C CNN
F 3 "" H 1100 6500 50  0001 C CNN
	1    1050 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	650  6400 650  6350
Wire Wire Line
	850  6400 650  6400
Wire Wire Line
	5600 7450 5500 7450
Wire Wire Line
	5500 7450 5500 7500
Wire Wire Line
	6800 5700 7300 5700
Wire Wire Line
	6700 5800 7300 5800
Wire Wire Line
	1150 3900 2950 3900
Wire Wire Line
	3100 3850 3000 3850
Wire Wire Line
	3000 3850 3000 4000
Wire Wire Line
	3000 4000 1150 4000
Wire Wire Line
	750  4350 1700 4350
Connection ~ 1100 4350
Wire Wire Line
	850  4650 1700 4650
Connection ~ 1100 4650
Connection ~ 1400 4350
Connection ~ 1400 4650
Wire Wire Line
	750  4300 750  4350
Wire Wire Line
	850  4300 850  4650
Wire Wire Line
	3100 3950 3100 4050
Wire Wire Line
	3500 2350 3800 2350
Connection ~ 3600 2350
Connection ~ 3700 2350
Wire Wire Line
	3700 1700 3700 2350
Wire Wire Line
	4000 2350 4600 2350
Connection ~ 4100 2350
Connection ~ 4300 2350
Connection ~ 4400 2350
Connection ~ 4500 2350
Wire Wire Line
	4100 2350 4200 2350
Connection ~ 4200 2350
Wire Wire Line
	4300 1700 4300 2350
Wire Wire Line
	2400 1700 3700 1700
Connection ~ 2650 1700
Connection ~ 2900 1700
Connection ~ 3150 1700
Wire Wire Line
	2400 2000 3400 2000
Connection ~ 3150 2000
Connection ~ 2900 2000
Connection ~ 2650 2000
Wire Wire Line
	6500 1700 4300 1700
Connection ~ 5000 1700
Connection ~ 5250 1700
Connection ~ 5500 1700
Connection ~ 5750 1700
Connection ~ 6000 1700
Connection ~ 6250 1700
Wire Wire Line
	4750 2000 6500 2000
Connection ~ 6250 2000
Connection ~ 6000 2000
Connection ~ 5750 2000
Connection ~ 5500 2000
Connection ~ 5250 2000
Connection ~ 5000 2000
Connection ~ 3400 1700
Connection ~ 4750 1700
Wire Wire Line
	1850 2650 2950 2650
Wire Wire Line
	2950 2650 2950 3050
Wire Wire Line
	2950 3050 3100 3050
Connection ~ 2000 2650
Wire Wire Line
	1400 2650 1550 2650
Wire Wire Line
	5950 950  6500 950 
Wire Wire Line
	5950 1100 6100 1100
Wire Wire Line
	6100 1100 6100 1250
Wire Wire Line
	6100 1250 6300 1250
Wire Wire Line
	1650 1300 3200 1300
Wire Wire Line
	2850 1000 3400 1000
Wire Wire Line
	2850 1100 2900 1100
Wire Wire Line
	2900 1100 2900 1000
Connection ~ 2900 1000
Connection ~ 2450 1300
Wire Wire Line
	6500 950  6500 1700
Connection ~ 6300 950 
Wire Wire Line
	3400 1000 3400 1700
Connection ~ 3200 1000
Wire Wire Line
	7300 4300 7300 4500
Connection ~ 7300 4400
Wire Wire Line
	8450 4400 8450 4500
Wire Wire Line
	7250 5600 7300 5600
Wire Wire Line
	7250 5900 7300 5900
Connection ~ 7250 5600
Wire Wire Line
	7250 6100 7300 6100
Connection ~ 7250 5900
Wire Wire Line
	8450 4600 9500 4600
Connection ~ 7250 6100
Wire Wire Line
	5700 3350 6800 3350
Wire Wire Line
	6800 3350 6800 5700
Wire Wire Line
	6700 3450 6700 5800
Wire Wire Line
	6700 3450 5700 3450
Wire Wire Line
	7300 4400 7200 4400
Wire Wire Line
	8450 4450 8550 4450
Connection ~ 8450 4450
Connection ~ 9200 1650
Wire Wire Line
	8450 5100 8450 5200
Connection ~ 6300 4850
Wire Wire Line
	7250 6250 8500 6250
Wire Wire Line
	4400 7450 4300 7450
Wire Wire Line
	4300 7450 4300 7500
Wire Wire Line
	1550 7350 1950 7350
Wire Wire Line
	1950 7450 1850 7450
Wire Wire Line
	1850 7450 1850 7500
Wire Wire Line
	2450 7350 2750 7350
Wire Wire Line
	2750 7450 2650 7450
Wire Wire Line
	2650 7450 2650 7500
Wire Wire Line
	3300 7350 3600 7350
Wire Wire Line
	3600 7450 3500 7450
Wire Wire Line
	3500 7450 3500 7500
Wire Wire Line
	8950 1650 9850 1650
Wire Wire Line
	1400 2450 1600 2450
Wire Wire Line
	3100 6250 3000 6250
Connection ~ 3500 6550
Wire Wire Line
	8600 5150 8450 5150
Connection ~ 8450 5150
Wire Wire Line
	8450 4900 9100 4900
Wire Wire Line
	9100 4900 9100 4800
Wire Wire Line
	9100 4800 9500 4800
Wire Wire Line
	8450 4800 9050 4800
Wire Wire Line
	9050 4800 9050 4700
Wire Wire Line
	9050 4700 9500 4700
Wire Wire Line
	1400 2450 1400 2650
Wire Wire Line
	1650 1000 2050 1000
Wire Wire Line
	4850 950  5050 950 
Wire Wire Line
	5050 950  5050 1100
Wire Wire Line
	6600 3050 6600 3150
Wire Wire Line
	5700 3050 6050 3050
Wire Wire Line
	6350 3050 6600 3050
Wire Wire Line
	6600 3550 6600 3650
Wire Wire Line
	5700 3550 6050 3550
Wire Wire Line
	6350 3550 6600 3550
Wire Wire Line
	5700 4050 6050 4050
Wire Wire Line
	6350 4050 6600 4050
Wire Wire Line
	5700 4550 6050 4550
Wire Wire Line
	6350 4550 6600 4550
Wire Wire Line
	6600 4050 6600 4150
Wire Wire Line
	6600 4550 6600 4650
Wire Wire Line
	2700 6550 4400 6550
Wire Wire Line
	3100 5950 3000 5950
Wire Wire Line
	3100 6050 1900 6050
Wire Wire Line
	3100 5850 1900 5850
Wire Wire Line
	3100 5750 1900 5750
Wire Wire Line
	1900 5750 1900 5550
Connection ~ 7250 4600
Wire Wire Line
	2700 5950 2700 6550
Connection ~ 2700 6250
Wire Wire Line
	3100 3450 3100 3300
Wire Wire Line
	3100 3300 2750 3300
Wire Wire Line
	2750 3300 2750 2800
Wire Wire Line
	2750 2800 2300 2800
Wire Wire Line
	2300 2800 2300 3350
Wire Wire Line
	2600 2900 2600 3350
Connection ~ 2300 2900
Wire Wire Line
	3050 3550 3100 3550
Wire Wire Line
	3050 3350 3050 3550
Wire Wire Line
	2700 3350 3050 3350
Connection ~ 2600 3200
Connection ~ 2300 3200
Wire Wire Line
	2300 3650 2600 3650
Wire Wire Line
	2450 3650 2450 3700
Connection ~ 2450 3650
Wire Wire Line
	3100 3250 2900 3250
Wire Wire Line
	2900 3250 2900 3400
Wire Wire Line
	2900 3800 2900 3700
Wire Wire Line
	2950 3900 2950 3750
Wire Wire Line
	2950 3750 3100 3750
Wire Wire Line
	700  7350 1100 7350
Wire Wire Line
	1100 7450 1000 7450
Wire Wire Line
	1000 7450 1000 7500
Wire Wire Line
	4400 7350 4250 7350
Wire Wire Line
	9500 4150 9500 4050
Wire Wire Line
	7250 4600 7250 6250
Wire Wire Line
	6550 6450 8600 6450
Wire Wire Line
	6050 4850 7050 4850
Wire Wire Line
	7050 4850 7050 5000
Wire Wire Line
	7050 5000 7300 5000
Wire Wire Line
	6550 6450 6550 4850
Connection ~ 6550 4850
Wire Wire Line
	8600 6450 8600 5150
Wire Wire Line
	6900 4600 7300 4600
Wire Wire Line
	9500 4050 6850 4050
Wire Wire Line
	6850 4050 6850 4800
Wire Wire Line
	6850 4800 7300 4800
Wire Wire Line
	8900 4100 8900 4550
Wire Wire Line
	8900 4100 6900 4100
Wire Wire Line
	6900 4100 6900 4600
Wire Wire Line
	9850 750  9800 750 
Wire Wire Line
	9750 850  9850 850 
Wire Wire Line
	9800 950  9850 950 
Wire Wire Line
	9800 750  9800 950 
Connection ~ 9800 850 
Wire Wire Line
	9750 850  9750 800 
Wire Wire Line
	11000 850  11050 850 
Wire Wire Line
	11050 850  11050 650 
Wire Wire Line
	11000 950  11150 950 
Wire Wire Line
	11150 950  11150 800 
Wire Wire Line
	11000 1550 11050 1550
Wire Wire Line
	11050 1650 11000 1650
Wire Wire Line
	11050 1550 11050 1650
Wire Wire Line
	11050 1600 11100 1600
Connection ~ 11050 1600
Wire Wire Line
	11000 1950 11100 1950
Wire Wire Line
	11100 2250 11000 2250
Wire Wire Line
	11100 2550 11000 2550
Wire Wire Line
	11000 1050 11100 1050
Wire Wire Line
	11100 1050 11100 2600
Connection ~ 11100 1950
Connection ~ 11100 2250
Connection ~ 11100 2550
Wire Wire Line
	9750 2550 9850 2550
Wire Wire Line
	9750 2350 9850 2350
Wire Wire Line
	9850 2050 9750 2050
Wire Wire Line
	9850 1050 9750 1050
Wire Wire Line
	9750 1050 9750 2600
Connection ~ 9750 2050
Connection ~ 9750 2350
Connection ~ 9750 2550
Wire Wire Line
	9850 1350 9700 1350
Wire Wire Line
	9700 1350 9700 1050
Wire Wire Line
	9700 1050 9600 1050
Wire Wire Line
	9500 1450 9850 1450
Wire Wire Line
	9850 2150 9200 2150
Wire Wire Line
	9200 2150 9200 2850
Wire Wire Line
	9200 2850 5700 2850
Wire Wire Line
	9850 2250 9300 2250
Wire Wire Line
	9300 2250 9300 2950
Wire Wire Line
	9300 2950 5700 2950
Wire Wire Line
	7300 4900 7250 4900
Connection ~ 7250 4900
Wire Wire Line
	9500 4550 9500 4350
Wire Wire Line
	9300 3800 9250 3800
Wire Wire Line
	8800 3800 8950 3800
Wire Wire Line
	9100 4250 9500 4250
Wire Wire Line
	8900 4550 9500 4550
Connection ~ 9200 4550
Wire Wire Line
	9100 4250 9100 3950
Wire Wire Line
	9100 3950 9300 3950
Wire Wire Line
	9300 3950 9300 3800
Connection ~ 9200 4250
Wire Wire Line
	9450 5250 8650 5250
Wire Wire Line
	8650 5250 8650 5500
Wire Wire Line
	8650 5500 8450 5500
Wire Wire Line
	9450 5350 8700 5350
Wire Wire Line
	8700 5350 8700 5600
Wire Wire Line
	8700 5600 8450 5600
Wire Wire Line
	9450 5450 8750 5450
Wire Wire Line
	8750 5450 8750 5700
Wire Wire Line
	8750 5700 8450 5700
Wire Wire Line
	9450 5550 8800 5550
Wire Wire Line
	8800 5550 8800 5800
Wire Wire Line
	8800 5800 8450 5800
Wire Wire Line
	9950 5550 9950 5900
Wire Wire Line
	9950 5900 8450 5900
Wire Wire Line
	9950 5450 10050 5450
Wire Wire Line
	10050 5450 10050 6000
Wire Wire Line
	10050 6000 8450 6000
Wire Wire Line
	9950 5350 10150 5350
Wire Wire Line
	10150 5350 10150 6100
Wire Wire Line
	10150 6100 8450 6100
Wire Wire Line
	9950 5250 10150 5250
Wire Wire Line
	1150 3700 1700 3700
Wire Wire Line
	7600 650  7600 850 
Wire Wire Line
	8650 650  8650 850 
Wire Wire Line
	2600 3200 2700 3200
Wire Wire Line
	2700 3200 2700 3350
Wire Wire Line
	2600 3650 2600 3800
Wire Wire Line
	2600 3800 2900 3800
$Comp
L +12V #PWR?
U 1 1 5A65EB25
P 650 6350
F 0 "#PWR?" H 650 6200 50  0001 C CNN
F 1 "+12V" H 650 6490 50  0000 C CNN
F 2 "" H 650 6350 50  0001 C CNN
F 3 "" H 650 6350 50  0001 C CNN
	1    650  6350
	1    0    0    -1  
$EndComp
Text GLabel 850  6600 0    60   Input ~ 0
VPCIE
Wire Wire Line
	1250 6500 1450 6500
Wire Wire Line
	5600 7350 5400 7350
Text GLabel 1450 6500 2    60   Input ~ 0
genPow
Text GLabel 5400 7350 0    60   Input ~ 0
genPow
$Comp
L R R?
U 1 1 5A6653B8
P 6200 4050
F 0 "R?" V 6280 4050 50  0000 C CNN
F 1 "4.7k" V 6200 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6130 4050 50  0001 C CNN
F 3 "" H 6200 4050 50  0001 C CNN
	1    6200 4050
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5A665459
P 6200 3550
F 0 "R?" V 6280 3550 50  0000 C CNN
F 1 "4.7k" V 6200 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6130 3550 50  0001 C CNN
F 3 "" H 6200 3550 50  0001 C CNN
	1    6200 3550
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5A6655FC
P 6200 3050
F 0 "R?" V 6280 3050 50  0000 C CNN
F 1 "4.7k" V 6200 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6130 3050 50  0001 C CNN
F 3 "" H 6200 3050 50  0001 C CNN
	1    6200 3050
	0    1    1    0   
$EndComp
$EndSCHEMATC
