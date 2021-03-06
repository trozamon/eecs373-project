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
Date "3 dec 2014"
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
P 2100 3500
F 0 "C2" H 2150 3600 40  0000 L CNN
F 1 "100u" H 2150 3400 40  0000 L CNN
F 2 "~" H 2200 3350 30  0000 C CNN
F 3 "~" H 2100 3500 300 0000 C CNN
	1    2100 3500
	0    -1   -1   0   
$EndComp
$Comp
L CAPAPOL C3
U 1 1 54684D4B
P 2100 3850
F 0 "C3" H 2150 3950 40  0000 L CNN
F 1 "100u" H 2150 3750 40  0000 L CNN
F 2 "~" H 2200 3700 30  0000 C CNN
F 3 "~" H 2100 3850 300 0000 C CNN
	1    2100 3850
	0    -1   -1   0   
$EndComp
$Comp
L CAPAPOL C4
U 1 1 54684D51
P 2100 4200
F 0 "C4" H 2150 4300 40  0000 L CNN
F 1 "100u" H 2150 4100 40  0000 L CNN
F 2 "~" H 2200 4050 30  0000 C CNN
F 3 "~" H 2100 4200 300 0000 C CNN
	1    2100 4200
	0    -1   -1   0   
$EndComp
$Comp
L CAPAPOL C5
U 1 1 54684D57
P 2100 4550
F 0 "C5" H 2150 4650 40  0000 L CNN
F 1 "100u" H 2150 4450 40  0000 L CNN
F 2 "~" H 2200 4400 30  0000 C CNN
F 3 "~" H 2100 4550 300 0000 C CNN
	1    2100 4550
	0    -1   -1   0   
$EndComp
$Comp
L BATTERY BT1
U 1 1 54686346
P 1000 4000
F 0 "BT1" H 1000 4200 50  0000 C CNN
F 1 "SOLAR_CELL" H 1000 3810 50  0000 C CNN
F 2 "~" H 1000 4000 60  0000 C CNN
F 3 "~" H 1000 4000 60  0000 C CNN
	1    1000 4000
	0    1    1    0   
$EndComp
$Comp
L CAPAPOL C6
U 1 1 547EB771
P 2100 4900
F 0 "C6" H 2150 5000 40  0000 L CNN
F 1 "100u" H 2150 4800 40  0000 L CNN
F 2 "~" H 2200 4750 30  0000 C CNN
F 3 "~" H 2100 4900 300 0000 C CNN
	1    2100 4900
	0    -1   -1   0   
$EndComp
$Comp
L CAPAPOL C7
U 1 1 547EB777
P 2100 5250
F 0 "C7" H 2150 5350 40  0000 L CNN
F 1 "100u" H 2150 5150 40  0000 L CNN
F 2 "~" H 2200 5100 30  0000 C CNN
F 3 "~" H 2100 5250 300 0000 C CNN
	1    2100 5250
	0    -1   -1   0   
$EndComp
$Comp
L CAPAPOL C8
U 1 1 547EB781
P 2100 5600
F 0 "C8" H 2150 5700 40  0000 L CNN
F 1 "100u" H 2150 5500 40  0000 L CNN
F 2 "~" H 2200 5450 30  0000 C CNN
F 3 "~" H 2100 5600 300 0000 C CNN
	1    2100 5600
	0    -1   -1   0   
$EndComp
Text Notes 600  2750 0    60   ~ 0
Solar Cell\n
Text Notes 1650 2750 0    60   ~ 0
Buffer
$Comp
L MN1382S U1
U 1 1 547EBA67
P 3150 4450
F 0 "U1" H 3150 4100 60  0000 C CNN
F 1 "MN1382S 2.0V" H 3150 4800 60  0000 C CNN
F 2 "" H 3150 4450 60  0000 C CNN
F 3 "" H 3150 4450 60  0000 C CNN
	1    3150 4450
	1    0    0    -1  
$EndComp
Text Notes 2550 2750 0    60   ~ 0
Voltage Monitor
$Comp
L TC54VC3002ECB713 U2
U 1 1 547EBB9F
P 4600 4950
F 0 "U2" H 4700 4650 60  0000 C CNN
F 1 "TC54VC 2.9V" H 4550 5300 60  0000 C CNN
F 2 "~" H 4550 4950 60  0000 C CNN
F 3 "~" H 4550 4950 60  0000 C CNN
	1    4600 4950
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 547EBC42
P 5100 5500
F 0 "R1" V 5180 5500 40  0000 C CNN
F 1 "1k" V 5107 5501 40  0000 C CNN
F 2 "~" V 5030 5500 30  0000 C CNN
F 3 "~" H 5100 5500 30  0000 C CNN
	1    5100 5500
	1    0    0    -1  
$EndComp
Text Notes 4000 2750 0    60   ~ 0
Overvoltage Discharge
$Comp
L NMOS U3
U 1 1 547EC0CC
P 5850 4450
F 0 "U3" H 6000 4200 60  0000 C CNN
F 1 "NMOS" H 5650 4650 60  0000 C CNN
F 2 "~" H 5850 4450 60  0000 C CNN
F 3 "~" H 5850 4450 60  0000 C CNN
	1    5850 4450
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 547EC10B
P 5600 5150
F 0 "R2" V 5680 5150 40  0000 C CNN
F 1 "100k" V 5607 5151 40  0000 C CNN
F 2 "~" V 5530 5150 30  0000 C CNN
F 3 "~" H 5600 5150 30  0000 C CNN
	1    5600 5150
	1    0    0    -1  
$EndComp
Text Notes 5450 2750 0    60   ~ 0
Enable
$Comp
L R R4
U 1 1 547EC2B4
P 6850 3550
F 0 "R4" V 6930 3550 40  0000 C CNN
F 1 "470k" V 6857 3551 40  0000 C CNN
F 2 "~" V 6780 3550 30  0000 C CNN
F 3 "~" H 6850 3550 30  0000 C CNN
	1    6850 3550
	1    0    0    -1  
$EndComp
$Comp
L NMOS U4
U 1 1 547EC2C3
P 6850 4250
F 0 "U4" H 7000 4000 60  0000 C CNN
F 1 "NMOS" H 6650 4450 60  0000 C CNN
F 2 "~" H 6850 4250 60  0000 C CNN
F 3 "~" H 6850 4250 60  0000 C CNN
	1    6850 4250
	1    0    0    -1  
$EndComp
$Comp
L PMOS U5
U 1 1 547EC3C5
P 7350 3900
F 0 "U5" H 7450 3750 60  0000 C CNN
F 1 "PMOS" H 7150 4050 60  0000 C CNN
F 2 "~" H 7350 3900 60  0000 C CNN
F 3 "~" H 7350 3900 60  0000 C CNN
	1    7350 3900
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 547EC3D4
P 6450 5150
F 0 "R3" V 6530 5150 40  0000 C CNN
F 1 "470k" V 6457 5151 40  0000 C CNN
F 2 "~" V 6380 5150 30  0000 C CNN
F 3 "~" H 6450 5150 30  0000 C CNN
	1    6450 5150
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 547EC3ED
P 7350 5100
F 0 "R5" V 7430 5100 40  0000 C CNN
F 1 "470k" V 7357 5101 40  0000 C CNN
F 2 "~" V 7280 5100 30  0000 C CNN
F 3 "~" H 7350 5100 30  0000 C CNN
	1    7350 5100
	1    0    0    -1  
$EndComp
Text Notes 6300 2750 0    60   ~ 0
Latch
$Comp
L MCP1640 U6
U 1 1 547EC976
P 8750 4250
F 0 "U6" H 9100 3800 60  0000 C CNN
F 1 "MCP1640" H 8500 4650 60  0000 C CNN
F 2 "" H 8750 4250 60  0000 C CNN
F 3 "" H 8750 4250 60  0000 C CNN
	1    8750 4250
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L1
U 1 1 547ECDC9
P 8750 3250
F 0 "L1" V 8700 3250 40  0000 C CNN
F 1 "4.7u" V 8850 3250 40  0000 C CNN
F 2 "~" H 8750 3250 60  0000 C CNN
F 3 "~" H 8750 3250 60  0000 C CNN
	1    8750 3250
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 547ECF2E
P 7850 3950
F 0 "C9" H 7850 4050 40  0000 L CNN
F 1 "4.7u" H 7856 3865 40  0000 L CNN
F 2 "~" H 7888 3800 30  0000 C CNN
F 3 "~" H 7850 3950 60  0000 C CNN
	1    7850 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 547ECF97
P 7850 4300
F 0 "#PWR01" H 7850 4300 30  0001 C CNN
F 1 "GND" H 7850 4230 30  0001 C CNN
F 2 "" H 7850 4300 60  0000 C CNN
F 3 "" H 7850 4300 60  0000 C CNN
	1    7850 4300
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 547ED0BA
P 9700 4250
F 0 "R7" V 9780 4250 40  0000 C CNN
F 1 "976k" V 9707 4251 40  0000 C CNN
F 2 "~" V 9630 4250 30  0000 C CNN
F 3 "~" H 9700 4250 30  0000 C CNN
	1    9700 4250
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 547ED0C9
P 9700 4850
F 0 "R8" V 9780 4850 40  0000 C CNN
F 1 "562k" V 9707 4851 40  0000 C CNN
F 2 "~" V 9630 4850 30  0000 C CNN
F 3 "~" H 9700 4850 30  0000 C CNN
	1    9700 4850
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 547ED31E
P 10000 4300
F 0 "C10" H 10000 4400 40  0000 L CNN
F 1 "10u" H 10006 4215 40  0000 L CNN
F 2 "~" H 10038 4150 30  0000 C CNN
F 3 "~" H 10000 4300 60  0000 C CNN
	1    10000 4300
	1    0    0    -1  
$EndComp
$Comp
L MCP1700 U7
U 1 1 547ED591
P 9600 3100
F 0 "U7" H 9700 2900 60  0000 C CNN
F 1 "MCP1700" H 9600 3300 60  0000 C CNN
F 2 "" H 9600 3100 60  0000 C CNN
F 3 "" H 9600 3100 60  0000 C CNN
	1    9600 3100
	1    0    0    -1  
$EndComp
Connection ~ 1900 2950
Wire Wire Line
	1000 2950 1000 3700
Wire Wire Line
	1000 2950 8750 2950
Wire Wire Line
	1000 4300 1000 5900
Wire Wire Line
	1000 5900 10800 5900
Wire Notes Line
	600  2800 600  6000
Wire Notes Line
	600  6000 1450 6000
Wire Notes Line
	1450 6000 1450 2800
Wire Notes Line
	1450 2800 600  2800
Wire Notes Line
	1650 2800 1650 6000
Wire Notes Line
	1650 6000 2450 6000
Wire Notes Line
	2450 6000 2450 2800
Wire Notes Line
	2450 2800 1650 2800
Connection ~ 2300 5900
Wire Notes Line
	2550 2800 2550 6000
Wire Notes Line
	2550 6000 3900 6000
Wire Notes Line
	3900 6000 3900 2800
Wire Notes Line
	3900 2800 2550 2800
Wire Wire Line
	3750 4450 5600 4450
Wire Wire Line
	4100 4450 4100 4850
Wire Wire Line
	4100 5900 4100 5050
Wire Wire Line
	5100 5900 5100 5750
Connection ~ 4100 5900
Wire Wire Line
	5100 5250 5100 4950
Wire Notes Line
	4000 6000 5350 6000
Wire Notes Line
	5350 6000 5350 2800
Wire Notes Line
	5350 2800 4000 2800
Wire Notes Line
	4000 2800 4000 6000
Connection ~ 4100 4450
Wire Wire Line
	5600 5900 5600 5400
Connection ~ 5100 5900
Wire Wire Line
	5600 4450 5600 4900
Wire Wire Line
	5850 4650 5850 6250
Connection ~ 5600 5900
Wire Notes Line
	5450 6000 5450 2800
Wire Notes Line
	5450 2800 6150 2800
Wire Notes Line
	6150 2800 6150 6000
Wire Notes Line
	6150 6000 5450 6000
Wire Wire Line
	6850 2950 6850 3300
Wire Wire Line
	6850 3800 6850 4050
Wire Wire Line
	5850 4250 5850 3900
Wire Wire Line
	5850 3900 7100 3900
Connection ~ 6850 3900
Wire Wire Line
	6850 5900 6850 4450
Connection ~ 5850 5900
Connection ~ 6850 2950
Wire Wire Line
	6450 4250 6450 4900
Wire Wire Line
	6450 4250 6600 4250
Connection ~ 6450 4600
Wire Wire Line
	6450 5400 6450 5900
Connection ~ 6450 5900
Connection ~ 6850 5900
Wire Notes Line
	6300 6000 6300 2800
Wire Notes Line
	6300 2800 7600 2800
Wire Notes Line
	7600 2800 7600 6000
Wire Notes Line
	7600 6000 6300 6000
Wire Wire Line
	7350 2950 7350 3700
Wire Wire Line
	6450 4600 7800 4600
Wire Wire Line
	7350 4100 7350 4850
Connection ~ 7350 4600
Wire Wire Line
	7350 5900 7350 5350
Wire Wire Line
	8050 2950 8050 4050
Connection ~ 7350 2950
Wire Wire Line
	7800 4600 7800 4450
Wire Wire Line
	7800 4450 8050 4450
Wire Wire Line
	8750 5900 8750 4900
Connection ~ 7350 5900
Connection ~ 8050 2950
Wire Wire Line
	8750 3550 8750 3600
Wire Wire Line
	8050 3750 7850 3750
Connection ~ 8050 3750
Wire Wire Line
	7850 4150 7850 4300
Wire Wire Line
	9450 4050 9600 4050
Wire Wire Line
	9600 4050 9600 4000
Wire Wire Line
	9600 4000 10350 4000
Wire Wire Line
	9450 4450 9600 4450
Wire Wire Line
	9600 4450 9600 4550
Wire Wire Line
	9600 4550 9700 4550
Wire Wire Line
	9700 4500 9700 4600
Connection ~ 9700 4550
Wire Wire Line
	9700 5900 9700 5100
Connection ~ 8750 5900
Wire Wire Line
	10000 4000 10000 4100
Connection ~ 9700 4000
Wire Wire Line
	10000 5900 10000 4500
Connection ~ 9700 5900
Wire Wire Line
	2700 4300 2700 2950
Connection ~ 2700 2950
Wire Wire Line
	2700 4600 2700 5900
Connection ~ 2700 5900
Wire Notes Line
	7650 2800 7650 6000
Wire Notes Line
	7650 6000 10250 6000
Wire Notes Line
	10250 6000 10250 2800
Wire Notes Line
	10250 2800 7650 2800
Text Notes 7650 2750 0    60   ~ 0
Output
Wire Wire Line
	9700 3850 9700 4000
Wire Wire Line
	9000 3850 9700 3850
$Comp
L GND #PWR02
U 1 1 547EDB28
P 9600 3650
F 0 "#PWR02" H 9600 3650 30  0001 C CNN
F 1 "GND" H 9600 3580 30  0001 C CNN
F 2 "" H 9600 3650 60  0000 C CNN
F 3 "" H 9600 3650 60  0000 C CNN
	1    9600 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 3650 9600 3550
$Comp
L CONN_3 K1
U 1 1 547EDBA9
P 10850 3550
F 0 "K1" V 10800 3550 50  0000 C CNN
F 1 "VSEL" V 10900 3550 40  0000 C CNN
F 2 "" H 10850 3550 60  0000 C CNN
F 3 "" H 10850 3550 60  0000 C CNN
	1    10850 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 3100 10350 3100
Wire Wire Line
	10350 3100 10350 3450
Wire Wire Line
	10350 3450 10500 3450
Wire Wire Line
	10350 4000 10350 3650
Wire Wire Line
	10350 3650 10500 3650
Connection ~ 10000 4000
Wire Wire Line
	10500 3550 10450 3550
Wire Wire Line
	10450 3550 10450 4150
$Comp
L CONN_6 P2
U 1 1 547EDDEB
P 10750 4550
F 0 "P2" V 10700 4550 60  0000 C CNN
F 1 "CONN_6" V 10800 4550 60  0000 C CNN
F 2 "" H 10750 4550 60  0000 C CNN
F 3 "" H 10750 4550 60  0000 C CNN
	1    10750 4550
	0    -1   -1   0   
$EndComp
$Comp
L CONN_2 P1
U 1 1 547EDFB9
P 9350 3550
F 0 "P1" V 9300 3550 40  0000 C CNN
F 1 "1V8_EN" V 9400 3550 40  0000 C CNN
F 2 "" H 9350 3550 60  0000 C CNN
F 3 "" H 9350 3550 60  0000 C CNN
	1    9350 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 3850 9000 3650
Wire Wire Line
	9000 2500 9000 3450
Wire Wire Line
	9000 3100 9050 3100
Wire Wire Line
	10450 4150 10350 4150
Wire Wire Line
	10350 4150 10350 5000
Wire Wire Line
	10350 5000 10600 5000
Wire Wire Line
	10600 5000 10600 4900
Wire Wire Line
	10800 5900 10800 4900
Connection ~ 10000 5900
$Comp
L R R6
U 1 1 547F11BA
P 9250 2500
F 0 "R6" V 9330 2500 40  0000 C CNN
F 1 "470k" V 9257 2501 40  0000 C CNN
F 2 "~" V 9180 2500 30  0000 C CNN
F 3 "~" H 9250 2500 30  0000 C CNN
	1    9250 2500
	0    -1   -1   0   
$EndComp
Connection ~ 9000 3100
$Comp
L GND #PWR03
U 1 1 547F1251
P 9500 2650
F 0 "#PWR03" H 9500 2650 30  0001 C CNN
F 1 "GND" H 9500 2580 30  0001 C CNN
F 2 "" H 9500 2650 60  0000 C CNN
F 3 "" H 9500 2650 60  0000 C CNN
	1    9500 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 2500 9500 2650
$Comp
L GND #PWR04
U 1 1 547F12E4
P 5850 6250
F 0 "#PWR04" H 5850 6250 30  0001 C CNN
F 1 "GND" H 5850 6180 30  0001 C CNN
F 2 "" H 5850 6250 60  0000 C CNN
F 3 "" H 5850 6250 60  0000 C CNN
	1    5850 6250
	1    0    0    -1  
$EndComp
$Comp
L CAPAPOL C1
U 1 1 54684D5D
P 2100 3150
F 0 "C1" H 2150 3250 40  0000 L CNN
F 1 "100u" H 2150 3050 40  0000 L CNN
F 2 "~" H 2200 3000 30  0000 C CNN
F 3 "~" H 2100 3150 300 0000 C CNN
	1    2100 3150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1900 2950 1900 5600
Connection ~ 1900 3150
Connection ~ 1900 3500
Connection ~ 1900 3850
Connection ~ 1900 4200
Connection ~ 1900 4550
Connection ~ 1900 5250
Connection ~ 1900 4900
Connection ~ 2300 5250
Connection ~ 2300 4550
Connection ~ 2300 4900
Wire Wire Line
	2300 3150 2300 5900
Connection ~ 2300 4200
Connection ~ 2300 3850
Connection ~ 2300 3500
Connection ~ 2300 5600
$EndSCHEMATC
