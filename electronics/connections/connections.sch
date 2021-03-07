EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 20
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
S 4700 1750 850  1350
U 6044A1E3
F0 "Jetson Nano" 50
F1 "Jeston Nano.sch" 50
F2 "PWR_JACK+" I L 4700 1850 50 
F3 "M2" I R 5550 2450 50 
F4 "PWR_JACK-" I L 4700 1950 50 
F5 "USB1" I R 5550 2050 50 
F6 "USB2" I R 5550 2150 50 
F7 "USB3" I R 5550 2250 50 
F8 "USB4" I R 5550 2350 50 
F9 "CAM0" I R 5550 1850 50 
F10 "CAM1" I R 5550 1950 50 
F11 "5V" I R 5550 2600 50 
F12 "GND" I R 5550 2700 50 
F13 "UART_2_TX" I R 5550 2800 50 
F14 "UART_2_RX" I R 5550 2900 50 
$EndSheet
$Sheet
S 6150 2400 650  300 
U 6044A4DF
F0 "STM32" 50
F1 "STM32.sch" 50
F2 "SHIELD" I R 6800 2550 50 
F3 "USB" I L 6150 2550 50 
$EndSheet
$Sheet
S 3450 1750 600  300 
U 6044A556
F0 "5V Step down 300W" 50
F1 "5V Step down 300W.sch" 50
F2 "IN+" I L 3450 1850 50 
F3 "IN-" I L 3450 1950 50 
F4 "OUT+" I R 4050 1850 50 
F5 "OUT-" I R 4050 1950 50 
$EndSheet
$Sheet
S 3450 1250 600  300 
U 6044A6E5
F0 "16V Step down 300W" 50
F1 "16V Step down 300W.sch" 50
F2 "IN+" I L 3450 1350 50 
F3 "IN-" I L 3450 1450 50 
F4 "OUT+" I R 4050 1350 50 
F5 "OUT-" I R 4050 1450 50 
$EndSheet
$Sheet
S 6150 2900 550  300 
U 6044A856
F0 "Lidar" 50
F1 "Lidar.sch" 50
F2 "USB_com" I L 6150 3000 50 
F3 "USB_pwr" I L 6150 3100 50 
$EndSheet
$Sheet
S 8800 5500 550  500 
U 6044A97A
F0 "Encoder left" 50
F1 "Encoder left.sch" 50
F2 "VCC" I L 8800 5600 50 
F3 "GND" I L 8800 5700 50 
F4 "SDA" I L 8800 5800 50 
F5 "SCL" I L 8800 5900 50 
$EndSheet
$Sheet
S 8800 4100 550  500 
U 6044A9E0
F0 "MPU9250" 50
F1 "MPU9250.sch" 50
F2 "GND" I L 8800 4300 50 
F3 "SCL" I L 8800 4400 50 
F4 "SDA" I L 8800 4500 50 
F5 "VCC" I L 8800 4200 50 
$EndSheet
$Sheet
S 8850 1750 550  300 
U 6044AA29
F0 "Motor left" 50
F1 "Motor left.sch" 50
F2 "RED" I L 8850 1850 50 
F3 "BLACK" I L 8850 1950 50 
$EndSheet
$Sheet
S 8850 1250 550  300 
U 6044AA4D
F0 "Motor right" 50
F1 "Motor right.sch" 50
F2 "RED" I L 8850 1350 50 
F3 "BLACK" I L 8850 1450 50 
$EndSheet
$Sheet
S 6150 3450 550  500 
U 6044AA86
F0 "GPS" 50
F1 "GPS.sch" 50
F2 "VCC" I L 6150 3550 50 
F3 "RX" I L 6150 3650 50 
F4 "TX" I L 6150 3750 50 
F5 "GND" I L 6150 3850 50 
$EndSheet
$Sheet
S 6150 2000 550  200 
U 6044AAC8
F0 "PI camera right" 50
F1 "PI camera right.sch" 50
F2 "RIBBON" I L 6150 2100 50 
$EndSheet
$Sheet
S 3450 2250 550  300 
U 6044AAF4
F0 "Voltometer" 50
F1 "Voltometer.sch" 50
F2 "+" I L 3450 2350 50 
F3 "-" I L 3450 2450 50 
$EndSheet
$Sheet
S 1650 2000 550  300 
U 6044AB51
F0 "12V battery left" 50
F1 "12V battery left.sch" 50
F2 "+" I R 2200 2100 50 
F3 "-" I R 2200 2200 50 
$EndSheet
$Sheet
S 1650 1500 550  300 
U 6044AB62
F0 "12V battery right" 50
F1 "12V battery right.sch" 50
F2 "+" I R 2200 1600 50 
F3 "-" I R 2200 1700 50 
$EndSheet
Wire Wire Line
	2200 1700 2700 1700
Wire Wire Line
	2700 1700 2700 2100
Wire Wire Line
	2700 2100 2200 2100
Wire Wire Line
	2200 1600 2700 1600
Wire Wire Line
	2700 1600 2700 1350
Wire Wire Line
	3200 1350 3350 1350
Wire Wire Line
	3350 1350 3350 1850
Wire Wire Line
	3350 1850 3450 1850
Connection ~ 3350 1350
Wire Wire Line
	3350 1350 3450 1350
Wire Wire Line
	3350 1850 3350 2350
Wire Wire Line
	3350 2350 3450 2350
Connection ~ 3350 1850
Wire Wire Line
	2200 2200 2700 2200
Wire Wire Line
	2700 2200 2700 2450
Wire Wire Line
	3300 2450 3300 1950
Wire Wire Line
	3300 1950 3450 1950
Connection ~ 3300 2450
Wire Wire Line
	3300 2450 3450 2450
Wire Wire Line
	3300 1950 3300 1450
Wire Wire Line
	3300 1450 3450 1450
Connection ~ 3300 1950
$Sheet
S 8800 4800 550  500 
U 6044A9AE
F0 "Encoder right" 50
F1 "Encoder right.sch" 50
F2 "VCC" I L 8800 4900 50 
F3 "GND" I L 8800 5000 50 
F4 "SDA" I L 8800 5100 50 
F5 "SCL" I L 8800 5200 50 
$EndSheet
Wire Wire Line
	8800 4900 8700 4900
Wire Wire Line
	8700 4900 8700 4200
Wire Wire Line
	8700 4200 8800 4200
Wire Wire Line
	8800 5600 8700 5600
Wire Wire Line
	8700 5600 8700 4900
Connection ~ 8700 4900
Wire Wire Line
	8800 5700 8650 5700
Wire Wire Line
	8650 5700 8650 5000
Wire Wire Line
	8650 5000 8800 5000
Wire Wire Line
	8650 5000 8650 4300
Wire Wire Line
	8650 4300 8800 4300
Connection ~ 8650 5000
Wire Wire Line
	8800 5800 8600 5800
Wire Wire Line
	8600 5800 8600 5100
Wire Wire Line
	8600 5100 8800 5100
Wire Wire Line
	8600 5100 8600 4500
Wire Wire Line
	8600 4500 8800 4500
Connection ~ 8600 5100
Wire Wire Line
	8800 5900 8550 5900
Wire Wire Line
	8550 5900 8550 5200
Wire Wire Line
	8550 5200 8800 5200
Wire Wire Line
	8550 5200 8550 4400
Wire Wire Line
	8550 4400 8800 4400
Connection ~ 8550 5200
$Comp
L Device:R R?
U 1 1 60456894
P 7750 3700
F 0 "R?" H 7820 3746 50  0000 L CNN
F 1 "R" H 7820 3655 50  0000 L CNN
F 2 "" V 7680 3700 50  0001 C CNN
F 3 "~" H 7750 3700 50  0001 C CNN
	1    7750 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 604570D2
P 7500 3700
F 0 "R?" H 7570 3746 50  0000 L CNN
F 1 "R" H 7570 3655 50  0000 L CNN
F 2 "" V 7430 3700 50  0001 C CNN
F 3 "~" H 7500 3700 50  0001 C CNN
	1    7500 3700
	1    0    0    -1  
$EndComp
$Sheet
S 7700 1250 600  800 
U 6045E05D
F0 "Motor shield" 50
F1 "Motor shield.sch" 50
F2 "+" I L 7700 1350 50 
F3 "-" I L 7700 1450 50 
F4 "A1" I R 8300 1600 50 
F5 "B1" I R 8300 1700 50 
F6 "A2" I R 8300 1850 50 
F7 "B2" I R 8300 1950 50 
F8 "SHIELD" I L 7700 1650 50 
$EndSheet
$Sheet
S 7400 2750 900  1200
U 6045FAE0
F0 "Big screw shield" 50
F1 "Big screw shield.sch" 50
F2 "SHIELD" I L 7400 2900 50 
F3 "PB9" I R 8300 2900 50 
F4 "PB8" I R 8300 3000 50 
F5 "3.3V_PIN" I R 8300 3100 50 
$EndSheet
$Sheet
S 7650 2250 650  300 
U 6045FB0F
F0 "Small screw shield" 50
F1 "Small screw shield.sch" 50
F2 "SHIELD" I L 7650 2400 50 
F3 "3.3V" I R 8300 2350 50 
F4 "GND" I R 8300 2450 50 
$EndSheet
Text Notes 7500 3450 0    50   ~ 0
solder directly \nto the pin
Wire Wire Line
	7500 3550 7500 3500
Wire Wire Line
	7500 3500 7750 3500
Wire Wire Line
	8350 3500 8350 3100
Wire Wire Line
	8350 3100 8300 3100
Wire Wire Line
	7750 3550 7750 3500
Connection ~ 7750 3500
Wire Wire Line
	7750 3500 8350 3500
Wire Wire Line
	8550 4400 7750 4400
Wire Wire Line
	7750 4400 7750 3850
Connection ~ 8550 4400
Wire Wire Line
	8600 4500 7500 4500
Wire Wire Line
	7500 4500 7500 3850
Connection ~ 8600 4500
Wire Wire Line
	8700 4200 8700 2350
Wire Wire Line
	8700 2350 8300 2350
Connection ~ 8700 4200
Wire Wire Line
	8650 4300 8650 2450
Wire Wire Line
	8650 2450 8300 2450
Connection ~ 8650 4300
Wire Wire Line
	8550 4400 8550 3000
Wire Wire Line
	8550 3000 8300 3000
Wire Wire Line
	8600 4500 8600 2900
Wire Wire Line
	8600 2900 8300 2900
Wire Wire Line
	7150 2400 7150 1650
Wire Wire Line
	7150 1650 7700 1650
Wire Wire Line
	7150 2400 7650 2400
Connection ~ 7150 2400
Wire Wire Line
	7150 2400 7150 2550
Wire Wire Line
	7150 2900 7400 2900
$Comp
L Switch:SW_Push SW?
U 1 1 60489A36
P 3000 1350
F 0 "SW?" H 3000 1635 50  0000 C CNN
F 1 "POWER SWITCH" H 3000 1544 50  0000 C CNN
F 2 "" H 3000 1550 50  0001 C CNN
F 3 "~" H 3000 1550 50  0001 C CNN
	1    3000 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1350 2800 1350
Wire Wire Line
	2700 2450 3300 2450
Wire Wire Line
	4050 1350 7700 1350
Wire Wire Line
	4050 1450 7700 1450
Wire Wire Line
	4050 1850 4700 1850
Wire Wire Line
	4050 1950 4700 1950
Connection ~ 7150 2550
Wire Wire Line
	7150 2550 7150 2900
Wire Wire Line
	6800 2550 7150 2550
$Sheet
S 6150 1600 550  200 
U 6044AAA7
F0 "PI camera left" 50
F1 "PI camera left.sch" 50
F2 "RIBBON" I L 6150 1700 50 
$EndSheet
Wire Wire Line
	5550 2050 6000 2050
Wire Wire Line
	6000 2050 6000 2550
Wire Wire Line
	6000 2550 6150 2550
Wire Wire Line
	8300 1600 8350 1600
Wire Wire Line
	8350 1600 8350 1350
Wire Wire Line
	8350 1350 8850 1350
Wire Wire Line
	8300 1700 8400 1700
Wire Wire Line
	8400 1700 8400 1450
Wire Wire Line
	8400 1450 8850 1450
Wire Wire Line
	8300 1850 8850 1850
Wire Wire Line
	8300 1950 8850 1950
Wire Wire Line
	5550 1850 5600 1850
Wire Wire Line
	5600 1850 5600 1700
Wire Wire Line
	5600 1700 6150 1700
Wire Wire Line
	5550 1950 6100 1950
Wire Wire Line
	6100 1950 6100 2100
Wire Wire Line
	6100 2100 6150 2100
Wire Wire Line
	5550 2150 5950 2150
Wire Wire Line
	5950 2150 5950 3000
Wire Wire Line
	5950 3000 6150 3000
Wire Wire Line
	5550 2600 5850 2600
Wire Wire Line
	5850 2600 5850 3550
Wire Wire Line
	5850 3550 6150 3550
Wire Wire Line
	5550 2700 5800 2700
Wire Wire Line
	5800 2700 5800 3850
Wire Wire Line
	5800 3850 6150 3850
Wire Wire Line
	5550 2800 5750 2800
Wire Wire Line
	5750 2800 5750 3650
Wire Wire Line
	5750 3650 6150 3650
Wire Wire Line
	5550 2900 5700 2900
Wire Wire Line
	5700 2900 5700 3750
Wire Wire Line
	5700 3750 6150 3750
$EndSCHEMATC
