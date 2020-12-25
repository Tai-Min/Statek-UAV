EESchema Schematic File Version 4
EELAYER 30 0
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
L Device:C_Small C5
U 1 1 5C962562
P 4500 5050
F 0 "C5" H 4592 5096 50  0000 L CNN
F 1 "100n" H 4592 5005 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 4500 5050 50  0001 C CNN
F 3 "~" H 4500 5050 50  0001 C CNN
	1    4500 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5C962A21
P 3350 4300
F 0 "C2" H 3442 4346 50  0000 L CNN
F 1 "10n" H 3442 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3350 4300 50  0001 C CNN
F 3 "~" H 3350 4300 50  0001 C CNN
	1    3350 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5C962651
P 3850 4300
F 0 "C4" H 3942 4346 50  0000 L CNN
F 1 "10n" H 3942 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3850 4300 50  0001 C CNN
F 3 "~" H 3850 4300 50  0001 C CNN
	1    3850 4300
	1    0    0    -1  
$EndComp
$Comp
L enkoder-rescue:am4096-enkoder U1
U 1 1 5C96229F
P 5300 3700
F 0 "U1" H 5100 4700 50  0000 C CNN
F 1 "am4096" H 5200 4650 50  0000 C CNN
F 2 "Package_SO:SSOP-28_5.3x10.2mm_P0.65mm" H 5450 4250 50  0001 C CNN
F 3 "" H 5450 4250 50  0001 C CNN
	1    5300 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4750 4750 4850
Wire Wire Line
	4750 4850 4500 4850
Wire Wire Line
	4500 4850 4500 4950
Wire Wire Line
	4850 4750 4850 4850
Wire Wire Line
	4200 4050 3850 4050
Wire Wire Line
	3850 4050 3850 4200
Wire Wire Line
	4200 3950 3350 3950
Wire Wire Line
	3350 3950 3350 4200
Wire Wire Line
	4850 4850 4950 4850
Wire Wire Line
	4950 4850 4950 5250
Wire Wire Line
	4950 5250 4500 5250
Wire Wire Line
	4500 5250 4500 5150
$Comp
L power:GND #PWR0102
U 1 1 5C965F6B
P 3600 4500
F 0 "#PWR0102" H 3600 4250 50  0001 C CNN
F 1 "GND" H 3605 4327 50  0000 C CNN
F 2 "" H 3600 4500 50  0001 C CNN
F 3 "" H 3600 4500 50  0001 C CNN
	1    3600 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5C965F84
P 4500 5300
F 0 "#PWR0103" H 4500 5050 50  0001 C CNN
F 1 "GND" H 4505 5127 50  0000 C CNN
F 2 "" H 4500 5300 50  0001 C CNN
F 3 "" H 4500 5300 50  0001 C CNN
	1    4500 5300
	1    0    0    -1  
$EndComp
Connection ~ 4500 5250
Wire Wire Line
	4500 5300 4500 5250
Wire Wire Line
	3350 4400 3350 4450
Wire Wire Line
	3350 4450 3600 4450
Connection ~ 3600 4450
Wire Wire Line
	3600 4450 3600 4500
Wire Wire Line
	3600 4450 3850 4450
Wire Wire Line
	3850 4450 3850 4400
$Comp
L Device:C_Small C8
U 1 1 5D7069C1
P 7750 3750
F 0 "C8" H 7842 3796 50  0000 L CNN
F 1 "100n" H 7842 3705 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 7750 3750 50  0001 C CNN
F 3 "~" H 7750 3750 50  0001 C CNN
	1    7750 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C9
U 1 1 5D7081C5
P 8200 3750
F 0 "C9" H 8288 3796 50  0000 L CNN
F 1 "2.2u" H 8288 3705 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3216-18_Kemet-A_Pad1.58x1.35mm_HandSolder" H 8200 3750 50  0001 C CNN
F 3 "~" H 8200 3750 50  0001 C CNN
	1    8200 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C3
U 1 1 5D70E13A
P 6400 3750
F 0 "C3" H 6488 3796 50  0000 L CNN
F 1 "2.2u" H 6488 3705 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3216-18_Kemet-A_Pad1.58x1.35mm_HandSolder" H 6400 3750 50  0001 C CNN
F 3 "~" H 6400 3750 50  0001 C CNN
	1    6400 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5D70E5BA
P 5950 3750
F 0 "C1" H 6042 3796 50  0000 L CNN
F 1 "100n" H 6042 3705 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5950 3750 50  0001 C CNN
F 3 "~" H 5950 3750 50  0001 C CNN
	1    5950 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C7
U 1 1 5D70E932
P 7300 3750
F 0 "C7" H 7388 3796 50  0000 L CNN
F 1 "2.2u" H 7388 3705 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3216-18_Kemet-A_Pad1.58x1.35mm_HandSolder" H 7300 3750 50  0001 C CNN
F 3 "~" H 7300 3750 50  0001 C CNN
	1    7300 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5D70ECC2
P 6850 3750
F 0 "C6" H 6942 3796 50  0000 L CNN
F 1 "100n" H 6942 3705 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 6850 3750 50  0001 C CNN
F 3 "~" H 6850 3750 50  0001 C CNN
	1    6850 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5D7136A4
P 7100 4100
F 0 "#PWR0101" H 7100 3850 50  0001 C CNN
F 1 "GND" H 7105 3927 50  0000 C CNN
F 2 "" H 7100 4100 50  0001 C CNN
F 3 "" H 7100 4100 50  0001 C CNN
	1    7100 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 3850 5950 3950
Wire Wire Line
	5950 3950 6400 3950
Wire Wire Line
	8200 3950 8200 3850
Wire Wire Line
	7750 3850 7750 3950
Connection ~ 7750 3950
Wire Wire Line
	7750 3950 8200 3950
Wire Wire Line
	7300 3850 7300 3950
Connection ~ 7300 3950
Wire Wire Line
	7300 3950 7750 3950
Wire Wire Line
	6850 3850 6850 3950
Connection ~ 6850 3950
Wire Wire Line
	6850 3950 7100 3950
Wire Wire Line
	6400 3850 6400 3950
Connection ~ 6400 3950
Wire Wire Line
	6400 3950 6850 3950
Wire Wire Line
	7100 3950 7100 4100
Connection ~ 7100 3950
Wire Wire Line
	7100 3950 7300 3950
Wire Wire Line
	6850 3650 6850 3550
Wire Wire Line
	6850 3550 7300 3550
Wire Wire Line
	7300 3550 7300 3650
Wire Wire Line
	7750 3650 7750 3550
Wire Wire Line
	7750 3550 8200 3550
Wire Wire Line
	8200 3550 8200 3650
Wire Wire Line
	5950 3650 5950 3550
Wire Wire Line
	5950 3550 6400 3550
Wire Wire Line
	6400 3550 6400 3650
Wire Wire Line
	5000 2600 5000 2700
Connection ~ 7750 3550
Connection ~ 6850 3550
Connection ~ 5950 3550
$Comp
L power:+3.3V #PWR0104
U 1 1 5D71ABD5
P 4800 2300
F 0 "#PWR0104" H 4800 2150 50  0001 C CNN
F 1 "+3.3V" H 4815 2473 50  0000 C CNN
F 2 "" H 4800 2300 50  0001 C CNN
F 3 "" H 4800 2300 50  0001 C CNN
	1    4800 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5D71C5D7
P 5500 4250
F 0 "#PWR0105" H 5500 4000 50  0001 C CNN
F 1 "GND" H 5505 4077 50  0000 C CNN
F 2 "" H 5500 4250 50  0001 C CNN
F 3 "" H 5500 4250 50  0001 C CNN
	1    5500 4250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5000 2600 5950 2600
Wire Wire Line
	5950 2600 5950 3550
Wire Wire Line
	4800 2500 6850 2500
Wire Wire Line
	4800 2500 4800 2700
Wire Wire Line
	6850 2500 6850 3550
Wire Wire Line
	4600 2400 7750 2400
Wire Wire Line
	7750 2400 7750 3550
Wire Wire Line
	4600 2400 4600 2700
Wire Wire Line
	4800 2300 4800 2500
Connection ~ 4800 2500
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5D73029D
P 6050 1850
F 0 "J1" H 6130 1842 50  0000 L CNN
F 1 "Conn_01x04" H 6130 1751 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 6050 1850 50  0001 C CNN
F 3 "~" H 6050 1850 50  0001 C CNN
	1    6050 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 4250 5500 4250
$Comp
L power:GND #PWR0106
U 1 1 5D736974
P 5750 1850
F 0 "#PWR0106" H 5750 1600 50  0001 C CNN
F 1 "GND" H 5755 1677 50  0000 C CNN
F 2 "" H 5750 1850 50  0001 C CNN
F 3 "" H 5750 1850 50  0001 C CNN
	1    5750 1850
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0107
U 1 1 5D736FD6
P 5750 1750
F 0 "#PWR0107" H 5750 1600 50  0001 C CNN
F 1 "+3.3V" V 5765 1878 50  0000 L CNN
F 2 "" H 5750 1750 50  0001 C CNN
F 3 "" H 5750 1750 50  0001 C CNN
	1    5750 1750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5850 2050 5750 2050
Wire Wire Line
	5850 1750 5750 1750
Text GLabel 5500 3350 2    50   Input ~ 0
SCL
Text GLabel 5500 3450 2    50   BiDi ~ 0
SDA
Wire Wire Line
	5400 3350 5500 3350
Wire Wire Line
	5400 3450 5500 3450
Text GLabel 5750 1950 0    50   BiDi ~ 0
SDA
Text GLabel 5750 2050 0    50   Output ~ 0
SCL
Wire Wire Line
	5750 1850 5850 1850
Wire Wire Line
	5750 1950 5850 1950
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5D82C065
P 7300 1950
F 0 "J2" H 7380 1942 50  0000 L CNN
F 1 "Conn_01x04" H 7380 1851 50  0000 L CNN
F 2 "TerminalBlock_MetzConnect:TerminalBlock_MetzConnect_Type059_RT06304HBWC_1x04_P3.50mm_Horizontal" H 7300 1950 50  0001 C CNN
F 3 "~" H 7300 1950 50  0001 C CNN
	1    7300 1950
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5D82C06B
P 7000 1850
F 0 "#PWR0108" H 7000 1600 50  0001 C CNN
F 1 "GND" H 7005 1677 50  0000 C CNN
F 2 "" H 7000 1850 50  0001 C CNN
F 3 "" H 7000 1850 50  0001 C CNN
	1    7000 1850
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0109
U 1 1 5D82C071
P 7000 1750
F 0 "#PWR0109" H 7000 1600 50  0001 C CNN
F 1 "+3.3V" V 7015 1878 50  0000 L CNN
F 2 "" H 7000 1750 50  0001 C CNN
F 3 "" H 7000 1750 50  0001 C CNN
	1    7000 1750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7100 2050 7000 2050
Wire Wire Line
	7100 1750 7000 1750
Text GLabel 7000 1950 0    50   BiDi ~ 0
SDA
Text GLabel 7000 2050 0    50   Output ~ 0
SCL
Wire Wire Line
	7000 1850 7100 1850
Wire Wire Line
	7000 1950 7100 1950
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 5FE6E9FA
P 3950 3650
F 0 "J4" H 4030 3642 50  0000 L CNN
F 1 "Conn_01x04" H 4030 3551 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 3950 3650 50  0001 C CNN
F 3 "~" H 3950 3650 50  0001 C CNN
	1    3950 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 3450 4150 3450
Wire Wire Line
	4200 3550 4150 3550
Wire Wire Line
	4200 3650 4150 3650
Wire Wire Line
	4200 3750 4150 3750
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 5FE74BC2
P 5650 3750
F 0 "J6" H 5730 3792 50  0000 L CNN
F 1 "Conn_01x03" H 5450 3550 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 5650 3750 50  0001 C CNN
F 3 "~" H 5650 3750 50  0001 C CNN
	1    5650 3750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 5FE7540B
P 5650 3050
F 0 "J5" H 5730 3042 50  0000 L CNN
F 1 "Conn_01x02" H 5400 3150 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 5650 3050 50  0001 C CNN
F 3 "~" H 5650 3050 50  0001 C CNN
	1    5650 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 5FE76B9E
P 6050 4250
F 0 "J7" H 6130 4242 50  0000 L CNN
F 1 "Conn_01x02" H 5800 4350 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 6050 4250 50  0001 C CNN
F 3 "~" H 6050 4250 50  0001 C CNN
	1    6050 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 5FE77758
P 3950 3150
F 0 "J3" H 4030 3192 50  0000 L CNN
F 1 "Conn_01x03" H 3750 2950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 3950 3150 50  0001 C CNN
F 3 "~" H 3950 3150 50  0001 C CNN
	1    3950 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 3050 4200 3050
Wire Wire Line
	4150 3150 4200 3150
Wire Wire Line
	4150 3250 4200 3250
Wire Wire Line
	5450 3050 5400 3050
Wire Wire Line
	5450 3150 5400 3150
Wire Wire Line
	5450 3650 5400 3650
Wire Wire Line
	5450 3750 5400 3750
Wire Wire Line
	5450 3850 5400 3850
Wire Wire Line
	5400 4050 5750 4050
Wire Wire Line
	5750 4050 5750 4250
Wire Wire Line
	5750 4250 5850 4250
Wire Wire Line
	5400 4450 5750 4450
Wire Wire Line
	5750 4450 5750 4350
Wire Wire Line
	5750 4350 5850 4350
$EndSCHEMATC
