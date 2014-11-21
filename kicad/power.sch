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
LIBS:special
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
LIBS:green_thumb_power
LIBS:power-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "21 nov 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CAPAPOL C2
U 1 1 54684D11
P 4800 3700
F 0 "C2" H 4850 3800 40  0000 L CNN
F 1 "100u" H 4850 3600 40  0000 L CNN
F 2 "~" H 4900 3550 30  0000 C CNN
F 3 "~" H 4800 3700 300 0000 C CNN
	1    4800 3700
	0    1    1    0   
$EndComp
$Comp
L CAPAPOL C3
U 1 1 54684D4B
P 4800 4050
F 0 "C3" H 4850 4150 40  0000 L CNN
F 1 "100u" H 4850 3950 40  0000 L CNN
F 2 "~" H 4900 3900 30  0000 C CNN
F 3 "~" H 4800 4050 300 0000 C CNN
	1    4800 4050
	0    1    1    0   
$EndComp
$Comp
L CAPAPOL C4
U 1 1 54684D51
P 4800 4400
F 0 "C4" H 4850 4500 40  0000 L CNN
F 1 "100u" H 4850 4300 40  0000 L CNN
F 2 "~" H 4900 4250 30  0000 C CNN
F 3 "~" H 4800 4400 300 0000 C CNN
	1    4800 4400
	0    1    1    0   
$EndComp
$Comp
L CAPAPOL C5
U 1 1 54684D57
P 4800 4750
F 0 "C5" H 4850 4850 40  0000 L CNN
F 1 "100u" H 4850 4650 40  0000 L CNN
F 2 "~" H 4900 4600 30  0000 C CNN
F 3 "~" H 4800 4750 300 0000 C CNN
	1    4800 4750
	0    1    1    0   
$EndComp
$Comp
L CAPAPOL C1
U 1 1 54684D5D
P 4800 3350
F 0 "C1" H 4850 3450 40  0000 L CNN
F 1 "100u" H 4850 3250 40  0000 L CNN
F 2 "~" H 4900 3200 30  0000 C CNN
F 3 "~" H 4800 3350 300 0000 C CNN
	1    4800 3350
	0    1    1    0   
$EndComp
$Comp
L XRP7659 U2
U 1 1 546852CB
P 7500 4000
F 0 "U2" H 7700 3750 60  0000 C CNN
F 1 "XRP7659" H 7450 4250 60  0000 C CNN
F 2 "" H 7550 3850 60  0000 C CNN
F 3 "" H 7550 3850 60  0000 C CNN
	1    7500 4000
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 546852F2
P 6250 4100
F 0 "C6" H 6250 4200 40  0000 L CNN
F 1 "10u" H 6256 4015 40  0000 L CNN
F 2 "~" H 6288 3950 30  0000 C CNN
F 3 "~" H 6250 4100 60  0000 C CNN
	1    6250 4100
	-1   0    0    1   
$EndComp
$Comp
L C C7
U 1 1 5468535A
P 8350 3800
F 0 "C7" H 8350 3900 40  0000 L CNN
F 1 "10n" H 8356 3715 40  0000 L CNN
F 2 "~" H 8388 3650 30  0000 C CNN
F 3 "~" H 8350 3800 60  0000 C CNN
	1    8350 3800
	-1   0    0    1   
$EndComp
$Comp
L DIODESCH D1
U 1 1 54685427
P 8250 4200
F 0 "D1" H 8250 4300 40  0000 C CNN
F 1 "2A/30V" H 8250 4100 40  0000 C CNN
F 2 "~" H 8250 4200 60  0000 C CNN
F 3 "~" H 8250 4200 60  0000 C CNN
	1    8250 4200
	0    -1   -1   0   
$EndComp
$Comp
L INDUCTOR_SMALL L1
U 1 1 54685483
P 8800 4000
F 0 "L1" H 8800 4100 50  0000 C CNN
F 1 "4.7u" H 8800 3950 50  0000 C CNN
F 2 "~" H 8800 4000 60  0000 C CNN
F 3 "~" H 8800 4000 60  0000 C CNN
	1    8800 4000
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 546854A6
P 9150 4450
F 0 "R3" V 9230 4450 40  0000 C CNN
F 1 "100k" V 9157 4451 40  0000 C CNN
F 2 "~" V 9080 4450 30  0000 C CNN
F 3 "~" H 9150 4450 30  0000 C CNN
	1    9150 4450
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 546854B3
P 9150 5150
F 0 "R4" V 9230 5150 40  0000 C CNN
F 1 "82k" V 9157 5151 40  0000 C CNN
F 2 "~" V 9080 5150 30  0000 C CNN
F 3 "~" H 9150 5150 30  0000 C CNN
	1    9150 5150
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 546854C3
P 9450 4200
F 0 "C8" H 9450 4300 40  0000 L CNN
F 1 "22u" H 9456 4115 40  0000 L CNN
F 2 "~" H 9488 4050 30  0000 C CNN
F 3 "~" H 9450 4200 60  0000 C CNN
	1    9450 4200
	-1   0    0    1   
$EndComp
Wire Wire Line
	5800 3900 6950 3900
Wire Wire Line
	8050 3900 8150 3900
Wire Wire Line
	8150 3900 8150 3600
Wire Wire Line
	8150 3600 8350 3600
Wire Wire Line
	8050 4000 8550 4000
Connection ~ 8250 4000
Connection ~ 8350 4000
Wire Wire Line
	9050 4000 9900 4000
Wire Wire Line
	9150 4000 9150 4200
Connection ~ 9150 4000
Wire Wire Line
	8250 4400 8250 4600
Wire Wire Line
	8050 4100 8050 4800
Wire Wire Line
	8050 4800 9150 4800
Wire Wire Line
	9150 4700 9150 4900
Connection ~ 9150 4800
$Comp
L GND #PWR4
U 1 1 54685704
P 8250 4600
F 0 "#PWR4" H 8250 4600 30  0001 C CNN
F 1 "GND" H 8250 4530 30  0001 C CNN
F 2 "" H 8250 4600 60  0000 C CNN
F 3 "" H 8250 4600 60  0000 C CNN
	1    8250 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR5
U 1 1 54685769
P 9450 4550
F 0 "#PWR5" H 9450 4550 30  0001 C CNN
F 1 "GND" H 9450 4480 30  0001 C CNN
F 2 "" H 9450 4550 60  0000 C CNN
F 3 "" H 9450 4550 60  0000 C CNN
	1    9450 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 4400 9450 4550
$Comp
L CONN_2 P1
U 1 1 54685872
P 10250 4100
F 0 "P1" V 10200 4100 40  0000 C CNN
F 1 "PWR_OUT" V 10300 4100 40  0000 C CNN
F 2 "" H 10250 4100 60  0000 C CNN
F 3 "" H 10250 4100 60  0000 C CNN
	1    10250 4100
	1    0    0    -1  
$EndComp
Connection ~ 9450 4000
Wire Wire Line
	9900 4200 9900 4500
Wire Wire Line
	9900 4500 9450 4500
Connection ~ 9450 4500
Wire Wire Line
	6950 4000 6550 4000
Text Label 6550 4000 0    60   ~ 0
PWR_OUT_EN
$Comp
L GND #PWR3
U 1 1 54685A14
P 6550 4500
F 0 "#PWR3" H 6550 4500 30  0001 C CNN
F 1 "GND" H 6550 4430 30  0001 C CNN
F 2 "" H 6550 4500 60  0000 C CNN
F 3 "" H 6550 4500 60  0000 C CNN
	1    6550 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 4300 6550 4300
Wire Wire Line
	6550 4100 6550 4500
Wire Wire Line
	6950 4100 6550 4100
Connection ~ 6550 4300
$Comp
L TC54VC3002ECB713 U1
U 1 1 54686187
P 5900 3250
F 0 "U1" H 6000 2950 60  0000 C CNN
F 1 "TC54VC3002ECB713" H 5850 3600 60  0000 C CNN
F 2 "" H 5850 3250 60  0000 C CNN
F 3 "" H 5850 3250 60  0000 C CNN
	1    5900 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2700 4600 4750
Connection ~ 4600 3700
Connection ~ 4600 4050
Connection ~ 4600 4400
Wire Wire Line
	5000 3350 5000 5100
Connection ~ 5000 4400
Connection ~ 5000 4050
Connection ~ 5000 3700
$Comp
L GND #PWR2
U 1 1 546862B7
P 5000 5100
F 0 "#PWR2" H 5000 5100 30  0001 C CNN
F 1 "GND" H 5000 5030 30  0001 C CNN
F 2 "" H 5000 5100 60  0000 C CNN
F 3 "" H 5000 5100 60  0000 C CNN
	1    5000 5100
	1    0    0    -1  
$EndComp
Connection ~ 5000 4750
$Comp
L BATTERY BT1
U 1 1 54686346
P 3700 4200
F 0 "BT1" H 3700 4400 50  0000 C CNN
F 1 "BATTERY" H 3700 4010 50  0000 C CNN
F 2 "~" H 3700 4200 60  0000 C CNN
F 3 "~" H 3700 4200 60  0000 C CNN
	1    3700 4200
	0    1    1    0   
$EndComp
$Comp
L +BATT #PWR1
U 1 1 54686355
P 3700 2850
F 0 "#PWR1" H 3700 2800 20  0001 C CNN
F 1 "+BATT" H 3700 2950 30  0000 C CNN
F 2 "" H 3700 2850 60  0000 C CNN
F 3 "" H 3700 2850 60  0000 C CNN
	1    3700 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 4500 3700 5000
Wire Wire Line
	3700 5000 5000 5000
Connection ~ 5000 5000
Wire Wire Line
	3700 2850 3700 3900
Connection ~ 4600 3350
Wire Wire Line
	5400 3350 5000 3350
Wire Wire Line
	6400 3250 6750 3250
Connection ~ 3700 3150
$Comp
L R R1
U 1 1 546865B5
P 5000 3150
F 0 "R1" V 5080 3150 40  0000 C CNN
F 1 "1M" V 5007 3151 40  0000 C CNN
F 2 "~" V 4930 3150 30  0000 C CNN
F 3 "~" H 5000 3150 30  0000 C CNN
	1    5000 3150
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 546865C2
P 5250 3800
F 0 "R2" V 5330 3800 40  0000 C CNN
F 1 "600k" V 5257 3801 40  0000 C CNN
F 2 "~" V 5180 3800 30  0000 C CNN
F 3 "~" H 5250 3800 30  0000 C CNN
	1    5250 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 3150 4750 3150
Connection ~ 4600 3150
Wire Wire Line
	5250 3150 5400 3150
Wire Wire Line
	5250 3150 5250 3550
Wire Wire Line
	5250 4050 5000 4050
Text Label 6750 3250 0    60   ~ 0
PWR_OUT_EN
Wire Wire Line
	4600 2700 5850 2700
Connection ~ 5300 3150
Connection ~ 6250 3900
Text Label 5800 3900 0    60   ~ 0
VIN
Text Label 5850 2700 0    60   ~ 0
VIN
$EndSCHEMATC
