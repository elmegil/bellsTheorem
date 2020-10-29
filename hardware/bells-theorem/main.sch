EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 8
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
L teensy:Teensy4.1 U1
U 1 1 5F831BBA
P 5150 3400
F 0 "U1" H 5150 5965 50  0000 C CNN
F 1 "Teensy4.1" H 5150 5874 50  0000 C CNN
F 2 "teensy:Teensy41" H 4750 3800 50  0001 C CNN
F 3 "" H 4750 3800 50  0001 C CNN
	1    5150 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F97FCF0
P 1250 6900
AR Path="/5F7954A8/5F97FCF0" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F97FCF0" Ref="#PWR03"  Part="1" 
F 0 "#PWR03" H 1250 6650 50  0001 C CNN
F 1 "GND" H 1255 6727 50  0000 C CNN
F 2 "" H 1250 6900 50  0001 C CNN
F 3 "" H 1250 6900 50  0001 C CNN
	1    1250 6900
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F97FCF6
P 10650 4100
AR Path="/5F7954A8/5F97FCF6" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F97FCF6" Ref="#PWR06"  Part="1" 
F 0 "#PWR06" H 10650 3850 50  0001 C CNN
F 1 "GND" H 10655 3927 50  0000 C CNN
F 2 "" H 10650 4100 50  0001 C CNN
F 3 "" H 10650 4100 50  0001 C CNN
	1    10650 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5F97FD02
P 10700 2100
AR Path="/5F7954A8/5F97FD02" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F97FD02" Ref="#PWR05"  Part="1" 
F 0 "#PWR05" H 10700 1950 50  0001 C CNN
F 1 "+12V" H 10715 2273 50  0000 C CNN
F 2 "" H 10700 2100 50  0001 C CNN
F 3 "" H 10700 2100 50  0001 C CNN
	1    10700 2100
	1    0    0    -1  
$EndComp
Text HLabel 10000 4750 3    50   Input ~ 0
ENCD_A
Text HLabel 10100 4750 3    50   Input ~ 0
ENCD_B
Text HLabel 9900 4750 3    50   Input ~ 0
ENCD_SW
Text HLabel 10400 4750 3    50   Input ~ 0
IN
Text HLabel 10300 4750 3    50   Input ~ 0
CVA
Text HLabel 10200 4750 3    50   Input ~ 0
CVB
Text HLabel 9700 2100 1    50   Output ~ 0
A
Text HLabel 9500 2100 1    50   Output ~ 0
B
Text HLabel 9800 2100 1    50   Output ~ 0
D
Text HLabel 3250 6800 3    50   Output ~ 0
AB
Text HLabel 2650 6800 3    50   Output ~ 0
CD
Text HLabel 9400 2100 1    50   Output ~ 0
DA
Text HLabel 3150 6800 3    50   Output ~ 0
ABC
Text HLabel 2750 6800 3    50   Output ~ 0
BCD
Text HLabel 2850 6800 3    50   Output ~ 0
CDA
Text HLabel 9300 2100 1    50   Output ~ 0
DAB
Text HLabel 9900 1400 1    50   Input ~ 0
ENCA_SW
Text HLabel 10000 1400 1    50   Input ~ 0
ENCA_B
Text HLabel 10100 1400 1    50   Input ~ 0
ENCA_A
$Comp
L Connectors_Generic:Conn_01x19 J2
U 1 1 5F85C3F5
P 10900 3100
F 0 "J2" H 10980 3142 50  0000 L CNN
F 1 "left" H 10980 3051 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x19_P2.54mm_Vertical" H 10900 3100 50  0001 C CNN
F 3 "~" H 10900 3100 50  0001 C CNN
	1    10900 3100
	1    0    0    -1  
$EndComp
$Comp
L Connectors_Generic:Conn_01x19 J3
U 1 1 5F85F4C1
P 1000 5900
F 0 "J3" H 1080 5942 50  0000 L CNN
F 1 "right" H 1080 5851 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x19_P2.54mm_Vertical" H 1000 5900 50  0001 C CNN
F 3 "~" H 1000 5900 50  0001 C CNN
	1    1000 5900
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F86539D
P 10250 2350
AR Path="/5F7954A8/5F86539D" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F86539D" Ref="#PWR047"  Part="1" 
F 0 "#PWR047" H 10250 2100 50  0001 C CNN
F 1 "GND" H 10255 2177 50  0000 C CNN
F 2 "" H 10250 2350 50  0001 C CNN
F 3 "" H 10250 2350 50  0001 C CNN
	1    10250 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 2350 10250 2300
Wire Wire Line
	10250 2300 10700 2300
Wire Wire Line
	10500 3850 10500 3900
Wire Wire Line
	10500 3900 10700 3900
Wire Wire Line
	10650 4100 10650 4000
Wire Wire Line
	10650 4000 10700 4000
$Comp
L power:GND #PWR?
U 1 1 5F866538
P 1450 5150
AR Path="/5F7954A8/5F866538" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F866538" Ref="#PWR055"  Part="1" 
F 0 "#PWR055" H 1450 4900 50  0001 C CNN
F 1 "GND" H 1455 4977 50  0000 C CNN
F 2 "" H 1450 5150 50  0001 C CNN
F 3 "" H 1450 5150 50  0001 C CNN
	1    1450 5150
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5F8674B5
P 1550 6650
AR Path="/5F7954A8/5F8674B5" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8674B5" Ref="#PWR056"  Part="1" 
F 0 "#PWR056" H 1550 6500 50  0001 C CNN
F 1 "+12V" H 1565 6823 50  0000 C CNN
F 2 "" H 1550 6650 50  0001 C CNN
F 3 "" H 1550 6650 50  0001 C CNN
	1    1550 6650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1250 4950 1250 5000
Wire Wire Line
	1250 5000 1200 5000
Wire Wire Line
	1450 5150 1450 5100
Wire Wire Line
	1450 5100 1200 5100
Wire Wire Line
	1550 6650 1550 6700
Wire Wire Line
	1550 6700 1200 6700
Wire Wire Line
	1250 6900 1250 6800
Wire Wire Line
	1250 6800 1200 6800
$Comp
L power:+5V #PWR?
U 1 1 5F8CA0BE
P 6450 1150
AR Path="/5F7954A8/5F8CA0BE" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8CA0BE" Ref="#PWR088"  Part="1" 
F 0 "#PWR088" H 6450 1000 50  0001 C CNN
F 1 "+5V" H 6465 1323 50  0000 C CNN
F 2 "" H 6450 1150 50  0001 C CNN
F 3 "" H 6450 1150 50  0001 C CNN
	1    6450 1150
	1    0    0    -1  
$EndComp
NoConn ~ 6250 1100
Wire Wire Line
	6250 1250 6450 1250
Wire Wire Line
	6450 1250 6450 1150
NoConn ~ 6250 3800
NoConn ~ 6250 3900
NoConn ~ 6250 4000
NoConn ~ 6250 4100
NoConn ~ 6250 4200
NoConn ~ 6250 4300
NoConn ~ 4050 3750
NoConn ~ 4050 3850
NoConn ~ 4050 4000
NoConn ~ 4050 4100
NoConn ~ 4050 4200
NoConn ~ 4050 4300
NoConn ~ 4050 4400
NoConn ~ 4850 4700
NoConn ~ 5000 4700
NoConn ~ 5150 4700
NoConn ~ 5300 4700
NoConn ~ 5450 4700
$Comp
L power:GND #PWR?
U 1 1 5F95B7C3
P 3850 3700
AR Path="/5F7954A8/5F95B7C3" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F95B7C3" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0130" H 3850 3450 50  0001 C CNN
F 1 "GND" H 3855 3527 50  0000 C CNN
F 2 "" H 3850 3700 50  0001 C CNN
F 3 "" H 3850 3700 50  0001 C CNN
	1    3850 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F95BE5C
P 6450 3300
AR Path="/5F7954A8/5F95BE5C" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F95BE5C" Ref="#PWR0131"  Part="1" 
F 0 "#PWR0131" H 6450 3050 50  0001 C CNN
F 1 "GND" H 6455 3127 50  0000 C CNN
F 2 "" H 6450 3300 50  0001 C CNN
F 3 "" H 6450 3300 50  0001 C CNN
	1    6450 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3300 6450 2650
Wire Wire Line
	6450 1350 6250 1350
Wire Wire Line
	6250 2650 6450 2650
Connection ~ 6450 2650
Wire Wire Line
	6450 2650 6450 1350
Wire Wire Line
	4050 1250 3850 1250
Wire Wire Line
	3850 1250 3850 3700
$Comp
L petelib:PI4MSD5V9540B U4
U 1 1 5F9B5B0A
P 7300 5200
F 0 "U4" H 7450 5500 50  0000 C CNN
F 1 "PI4MSD5V9540B" H 7400 4900 50  0000 C CNN
F 2 "Package_SO:MSOP-8_3x3mm_P0.65mm" H 7350 5200 50  0001 C CNN
F 3 "" H 7350 5200 50  0001 C CNN
	1    7300 5200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0105
U 1 1 5F9D2FC7
P 7300 4900
F 0 "#PWR0105" H 7300 4750 50  0001 C CNN
F 1 "+3.3V" H 7315 5073 50  0000 C CNN
F 2 "" H 7300 4900 50  0001 C CNN
F 3 "" H 7300 4900 50  0001 C CNN
	1    7300 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F9D39B6
P 7300 5500
AR Path="/5F7954A8/5F9D39B6" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9D39B6" Ref="#PWR0106"  Part="1" 
F 0 "#PWR0106" H 7300 5250 50  0001 C CNN
F 1 "GND" H 7305 5327 50  0000 C CNN
F 2 "" H 7300 5500 50  0001 C CNN
F 3 "" H 7300 5500 50  0001 C CNN
	1    7300 5500
	1    0    0    -1  
$EndComp
$Sheet
S 3350 5050 1100 1750
U 5F9DE87B
F0 "rightI2C" 50
F1 "rightI2C.sch" 50
F2 "AB" O L 3350 5500 50 
F3 "CD" O L 3350 6100 50 
F4 "ABC" O L 3350 5600 50 
F5 "BCD" O L 3350 6000 50 
F6 "CDA" O L 3350 5900 50 
F7 "right_SCL" I R 4450 6300 50 
F8 "right_SDA" I R 4450 6400 50 
F9 "~LAT" I R 4450 6200 50 
F10 "BC" O L 3350 5800 50 
F11 "ABCD" O L 3350 5700 50 
$EndSheet
NoConn ~ 4050 2650
NoConn ~ 6250 1450
Wire Wire Line
	6850 5250 6750 5250
Wire Wire Line
	6550 5250 6550 2050
Wire Wire Line
	6550 2050 6250 2050
Wire Wire Line
	6650 5150 6650 1950
Wire Wire Line
	6650 1950 6250 1950
Wire Wire Line
	4900 6400 4450 6400
Wire Wire Line
	4450 6300 5000 6300
Wire Wire Line
	3350 5500 3250 5500
Wire Wire Line
	3350 5600 3150 5600
Wire Wire Line
	3350 5700 3050 5700
Wire Wire Line
	3350 5800 2950 5800
Wire Wire Line
	3350 5900 2850 5900
Wire Wire Line
	3350 6000 2750 6000
Wire Wire Line
	3350 6100 2650 6100
Wire Wire Line
	3250 6800 3250 5500
Connection ~ 3250 5500
Wire Wire Line
	3250 5500 1200 5500
Wire Wire Line
	3150 6800 3150 5600
Connection ~ 3150 5600
Wire Wire Line
	3150 5600 1200 5600
Wire Wire Line
	3050 6800 3050 5700
Connection ~ 3050 5700
Wire Wire Line
	3050 5700 1200 5700
Wire Wire Line
	2950 6800 2950 5800
Connection ~ 2950 5800
Wire Wire Line
	2950 5800 1200 5800
Wire Wire Line
	2850 6800 2850 5900
Connection ~ 2850 5900
Wire Wire Line
	2850 5900 1200 5900
Wire Wire Line
	2750 6800 2750 6000
Connection ~ 2750 6000
Wire Wire Line
	2750 6000 1200 6000
Wire Wire Line
	2650 6800 2650 6100
Connection ~ 2650 6100
Wire Wire Line
	2650 6100 1200 6100
Wire Wire Line
	1200 6200 2150 6200
Wire Wire Line
	1200 6300 2050 6300
Wire Wire Line
	1200 6400 1950 6400
Wire Wire Line
	1200 6500 1850 6500
NoConn ~ 6250 2550
Wire Wire Line
	10700 2100 10700 2200
Text HLabel 9600 2100 1    50   Output ~ 0
C
Wire Wire Line
	9900 2600 10700 2600
Wire Wire Line
	10000 2500 10700 2500
Wire Wire Line
	10100 2400 10700 2400
Wire Wire Line
	10100 3100 10700 3100
Wire Wire Line
	10000 3000 10700 3000
Wire Wire Line
	9900 2900 10700 2900
NoConn ~ 4050 2950
NoConn ~ 4050 3050
NoConn ~ 4050 3150
NoConn ~ 4050 3250
NoConn ~ 4050 3350
NoConn ~ 4050 3450
NoConn ~ 4050 3550
NoConn ~ 6250 3150
NoConn ~ 6250 3250
NoConn ~ 6250 3350
NoConn ~ 6250 3450
Wire Wire Line
	8100 2250 7850 2250
$Sheet
S 8100 2000 1100 1800
U 5F9DEAE4
F0 "LeftI2C" 50
F1 "leftI2C.sch" 50
F2 "A" O R 9200 3500 50 
F3 "B" O R 9200 3300 50 
F4 "C" O R 9200 3200 50 
F5 "D" O R 9200 3400 50 
F6 "DA" O R 9200 2700 50 
F7 "DAB" O R 9200 2800 50 
F8 "left_SCL" I L 8100 2250 50 
F9 "left_SDA" I L 8100 2350 50 
F10 "~LAT" I L 8100 2450 50 
$EndSheet
$Comp
L device:R_Small_US R100
U 1 1 5FAD1EF6
P 6750 4950
F 0 "R100" H 6550 5000 50  0000 L CNN
F 1 "4.7K" H 6500 4900 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6750 4950 50  0001 C CNN
F 3 "~" H 6750 4950 50  0001 C CNN
	1    6750 4950
	1    0    0    -1  
$EndComp
$Comp
L device:R_Small_US R101
U 1 1 5FAD2A5C
P 6850 4950
F 0 "R101" H 6900 5100 50  0000 L CNN
F 1 "4.7K" H 6900 5000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 4950 50  0001 C CNN
F 3 "~" H 6850 4950 50  0001 C CNN
	1    6850 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 5150 6650 5150
Wire Wire Line
	6850 5050 6850 5150
Connection ~ 6850 5150
Wire Wire Line
	6750 5050 6750 5250
Connection ~ 6750 5250
Wire Wire Line
	6750 5250 6550 5250
$Comp
L power:+3.3V #PWR089
U 1 1 5FAF76CE
P 6750 4850
F 0 "#PWR089" H 6750 4700 50  0001 C CNN
F 1 "+3.3V" H 6765 5023 50  0000 C CNN
F 2 "" H 6750 4850 50  0001 C CNN
F 3 "" H 6750 4850 50  0001 C CNN
	1    6750 4850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR090
U 1 1 5FAF7CA2
P 6850 4850
F 0 "#PWR090" H 6850 4700 50  0001 C CNN
F 1 "+3.3V" H 6865 5023 50  0000 C CNN
F 2 "" H 6850 4850 50  0001 C CNN
F 3 "" H 6850 4850 50  0001 C CNN
	1    6850 4850
	1    0    0    -1  
$EndComp
$Comp
L device:R_Small_US R102
U 1 1 5FAFAC0E
P 5000 6050
F 0 "R102" H 4800 6100 50  0000 L CNN
F 1 "4.7K" H 4750 6000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5000 6050 50  0001 C CNN
F 3 "~" H 5000 6050 50  0001 C CNN
	1    5000 6050
	-1   0    0    -1  
$EndComp
$Comp
L device:R_Small_US R103
U 1 1 5FAFAC14
P 4900 6050
F 0 "R103" H 4950 6200 50  0000 L CNN
F 1 "4.7K" H 4950 6100 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4900 6050 50  0001 C CNN
F 3 "~" H 4900 6050 50  0001 C CNN
	1    4900 6050
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR091
U 1 1 5FAFAC1A
P 5000 5950
F 0 "#PWR091" H 5000 5800 50  0001 C CNN
F 1 "+3.3V" H 5015 6123 50  0000 C CNN
F 2 "" H 5000 5950 50  0001 C CNN
F 3 "" H 5000 5950 50  0001 C CNN
	1    5000 5950
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR092
U 1 1 5FAFAC20
P 4900 5950
F 0 "#PWR092" H 4900 5800 50  0001 C CNN
F 1 "+3.3V" H 4915 6123 50  0000 C CNN
F 2 "" H 4900 5950 50  0001 C CNN
F 3 "" H 4900 5950 50  0001 C CNN
	1    4900 5950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5000 6150 5000 6300
Wire Wire Line
	4900 6150 4900 6400
$Comp
L device:R_Small_US R104
U 1 1 5FB23DF1
P 7850 2100
F 0 "R104" H 7650 2150 50  0000 L CNN
F 1 "4.7K" H 7600 2050 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7850 2100 50  0001 C CNN
F 3 "~" H 7850 2100 50  0001 C CNN
	1    7850 2100
	1    0    0    -1  
$EndComp
$Comp
L device:R_Small_US R105
U 1 1 5FB23DF7
P 7950 2100
F 0 "R105" H 8000 2250 50  0000 L CNN
F 1 "4.7K" H 8000 2150 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7950 2100 50  0001 C CNN
F 3 "~" H 7950 2100 50  0001 C CNN
	1    7950 2100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR093
U 1 1 5FB23DFD
P 7850 2000
F 0 "#PWR093" H 7850 1850 50  0001 C CNN
F 1 "+3.3V" H 7865 2173 50  0000 C CNN
F 2 "" H 7850 2000 50  0001 C CNN
F 3 "" H 7850 2000 50  0001 C CNN
	1    7850 2000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR094
U 1 1 5FB23E03
P 7950 2000
F 0 "#PWR094" H 7950 1850 50  0001 C CNN
F 1 "+3.3V" H 7965 2173 50  0000 C CNN
F 2 "" H 7950 2000 50  0001 C CNN
F 3 "" H 7950 2000 50  0001 C CNN
	1    7950 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 2200 7850 2250
Wire Wire Line
	7950 2200 7950 2350
Wire Wire Line
	7950 2350 8100 2350
Wire Wire Line
	10100 3100 10100 4150
Wire Wire Line
	10000 3000 10000 4050
Wire Wire Line
	9900 2900 9900 3950
Wire Wire Line
	7850 2250 7850 5050
Wire Wire Line
	7850 5050 7750 5050
Connection ~ 7850 2250
Wire Wire Line
	7950 2350 7950 5150
Wire Wire Line
	7950 5150 7750 5150
Connection ~ 7950 2350
Wire Wire Line
	1200 6600 1750 6600
Wire Wire Line
	1750 6600 1750 6800
Wire Wire Line
	1850 6500 1850 6800
Wire Wire Line
	1950 6400 1950 6800
Wire Wire Line
	2050 6300 2050 6800
Wire Wire Line
	2150 6200 2150 6800
Text HLabel 2150 6800 3    50   Input ~ 0
ENCC_A
Text HLabel 2050 6800 3    50   Input ~ 0
ENCC_B
Text HLabel 1950 6800 3    50   Input ~ 0
ENCC_SW
Text HLabel 1850 6800 3    50   Input ~ 0
CVC
Text HLabel 1750 6800 3    50   Input ~ 0
CVD
Text HLabel 2550 5300 2    50   Input ~ 0
ENCB_A
Text HLabel 2550 5200 2    50   Input ~ 0
ENCB_SW
Text HLabel 2550 5400 2    50   Input ~ 0
ENCB_B
Wire Wire Line
	1200 5400 2450 5400
Wire Wire Line
	1200 5200 2250 5200
Wire Wire Line
	1200 5300 2350 5300
Wire Wire Line
	7750 5350 7850 5350
Wire Wire Line
	7850 5350 7850 6400
Wire Wire Line
	7850 6400 4900 6400
Connection ~ 4900 6400
Wire Wire Line
	7750 5250 7950 5250
Wire Wire Line
	7950 5250 7950 6300
Wire Wire Line
	7950 6300 5000 6300
Connection ~ 5000 6300
Wire Wire Line
	7750 3550 7750 2450
Wire Wire Line
	7750 2450 8100 2450
Wire Wire Line
	6350 3550 6350 6200
Wire Wire Line
	6350 6200 4450 6200
Wire Wire Line
	6350 3550 7750 3550
NoConn ~ 6250 2750
Wire Wire Line
	10400 3800 10700 3800
Wire Wire Line
	10100 4150 7050 4150
Wire Wire Line
	7050 4150 7050 2450
Wire Wire Line
	7050 2450 6250 2450
Connection ~ 10100 4150
Wire Wire Line
	10100 4150 10100 4750
Wire Wire Line
	10000 4050 7150 4050
Wire Wire Line
	7150 4050 7150 2350
Wire Wire Line
	7150 2350 6250 2350
Connection ~ 10000 4050
Wire Wire Line
	10000 4050 10000 4750
Wire Wire Line
	9900 3950 7250 3950
Wire Wire Line
	7250 3950 7250 2250
Wire Wire Line
	7250 2250 6250 2250
Connection ~ 9900 3950
Wire Wire Line
	9900 3950 9900 4750
NoConn ~ 6250 2150
NoConn ~ 6250 1850
Connection ~ 1950 6400
Connection ~ 2050 6300
Connection ~ 2150 6200
Connection ~ 2250 5200
Wire Wire Line
	2250 5200 2550 5200
Connection ~ 2350 5300
Wire Wire Line
	2350 5300 2550 5300
Connection ~ 2450 5400
Wire Wire Line
	2450 5400 2550 5400
NoConn ~ 4050 2050
NoConn ~ 4050 2150
NoConn ~ 4050 2250
NoConn ~ 4050 2350
NoConn ~ 4050 2450
NoConn ~ 4050 2550
$Comp
L power:+3.3V #PWR0198
U 1 1 5FA78B95
P 10500 3850
F 0 "#PWR0198" H 10500 3700 50  0001 C CNN
F 1 "+3.3V" H 10515 4023 50  0000 C CNN
F 2 "" H 10500 3850 50  0001 C CNN
F 3 "" H 10500 3850 50  0001 C CNN
	1    10500 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0199
U 1 1 5FA79550
P 1250 4950
F 0 "#PWR0199" H 1250 4800 50  0001 C CNN
F 1 "+3.3V" H 1265 5123 50  0000 C CNN
F 2 "" H 1250 4950 50  0001 C CNN
F 3 "" H 1250 4950 50  0001 C CNN
	1    1250 4950
	1    0    0    -1  
$EndComp
Text HLabel 2950 6800 3    50   Output ~ 0
BC
Text HLabel 3050 6800 3    50   Output ~ 0
ABCD
Wire Wire Line
	9900 1400 9900 1750
Wire Wire Line
	10000 1400 10000 1650
Wire Wire Line
	10100 1400 10100 1550
Wire Wire Line
	6250 1550 10100 1550
Connection ~ 10100 1550
Wire Wire Line
	10100 1550 10100 2400
Wire Wire Line
	6250 1650 10000 1650
Connection ~ 10000 1650
Wire Wire Line
	10000 1650 10000 2500
Wire Wire Line
	6250 1750 9900 1750
Connection ~ 9900 1750
Wire Wire Line
	9900 1750 9900 2600
NoConn ~ 6250 3550
Wire Wire Line
	7750 2450 7750 650 
Wire Wire Line
	7750 650  3950 650 
Wire Wire Line
	3950 650  3950 1350
Wire Wire Line
	3950 1350 4050 1350
Connection ~ 7750 2450
Wire Wire Line
	4050 1950 1950 1950
Wire Wire Line
	1950 1950 1950 6400
Wire Wire Line
	4050 1850 2050 1850
Wire Wire Line
	2050 1850 2050 6300
Wire Wire Line
	4050 1750 2150 1750
Wire Wire Line
	2150 1750 2150 6200
Wire Wire Line
	4050 1650 2450 1650
Wire Wire Line
	2450 1650 2450 5400
Wire Wire Line
	4050 1550 2350 1550
Wire Wire Line
	2350 1550 2350 5300
Wire Wire Line
	4050 1450 2250 1450
Wire Wire Line
	2250 1450 2250 5200
Wire Wire Line
	9200 2700 9400 2700
Wire Wire Line
	9200 2800 9300 2800
Wire Wire Line
	9300 2100 9300 2800
Connection ~ 9300 2800
Wire Wire Line
	9300 2800 10700 2800
Wire Wire Line
	9400 2100 9400 2700
Connection ~ 9400 2700
Wire Wire Line
	9400 2700 10700 2700
Wire Wire Line
	9500 2100 9500 3300
Wire Wire Line
	9600 2100 9600 3200
Wire Wire Line
	9700 2100 9700 3500
Wire Wire Line
	9800 2100 9800 3400
Wire Wire Line
	10400 3800 10400 4550
$Sheet
S 2700 2550 950  750 
U 5F8906B5
F0 "scaleright" 50
F1 "scaleright.sch" 50
F2 "CVC_out" O R 3650 2750 50 
F3 "CVD_out" O R 3650 2850 50 
F4 "CVC_in" I L 2700 2750 50 
F5 "CVD_in" I L 2700 2850 50 
$EndSheet
$Sheet
S 8500 4250 1100 750 
U 5F8909DB
F0 "scaleleft" 50
F1 "scaleleft.sch" 50
F2 "CVA_in" I R 9600 4450 50 
F3 "CVB_in" I R 9600 4350 50 
F4 "IN_in" I R 9600 4550 50 
F5 "CVA_out" O L 8500 4450 50 
F6 "CVB_out" O L 8500 4350 50 
F7 "IN_out" O L 8500 4550 50 
$EndSheet
Wire Wire Line
	9600 4550 10400 4550
Connection ~ 10400 4550
Wire Wire Line
	10400 4550 10400 4750
Wire Wire Line
	9600 4450 10300 4450
Wire Wire Line
	10300 4450 10300 4750
Wire Wire Line
	9600 4350 10200 4350
Wire Wire Line
	10200 4350 10200 4750
Wire Wire Line
	8500 4350 6950 4350
Wire Wire Line
	6950 4350 6950 2850
Wire Wire Line
	6950 2850 6250 2850
Wire Wire Line
	8500 4450 6850 4450
Wire Wire Line
	6850 4450 6850 2950
Wire Wire Line
	6850 2950 6250 2950
Wire Wire Line
	8500 4550 6750 4550
Wire Wire Line
	6750 4550 6750 3050
Wire Wire Line
	6750 3050 6250 3050
Wire Wire Line
	1850 6500 1850 2750
Wire Wire Line
	1850 2750 2700 2750
Connection ~ 1850 6500
Wire Wire Line
	3650 2750 4050 2750
Wire Wire Line
	1750 6600 1750 2850
Wire Wire Line
	1750 2850 2700 2850
Connection ~ 1750 6600
Wire Wire Line
	3650 2850 4050 2850
Wire Wire Line
	9200 3200 9600 3200
Connection ~ 9600 3200
Wire Wire Line
	9600 3200 10700 3200
Wire Wire Line
	9200 3300 9500 3300
Connection ~ 9500 3300
Wire Wire Line
	9500 3300 10700 3300
Wire Wire Line
	9200 3400 9800 3400
Connection ~ 9800 3400
Wire Wire Line
	9800 3400 10700 3400
Wire Wire Line
	9200 3500 9700 3500
Connection ~ 9700 3500
Wire Wire Line
	9700 3500 10700 3500
Wire Wire Line
	10700 3600 10200 3600
Wire Wire Line
	10200 3600 10200 4350
Connection ~ 10200 4350
Wire Wire Line
	10700 3700 10300 3700
Wire Wire Line
	10300 3700 10300 4450
Connection ~ 10300 4450
Text Notes 8150 6800 0    50   ~ 0
notes from Jim Matheson:\non here you can see you can get 12 bit pwm at 36 khz\n\nhttps://www.pjrc.com/teensy/td_pulse.html\nPulse Width and Tone on Teensy with Arduino\npjrc.com\n\nand sending an “analogwrite” to a pin is easier and faster than i2c\n\nor you could get a 8 channel spi dac\n\nyou can save about 40mA by using a recom switching 7805 replacement\n\nRecom: R-78?\n\nthere are even pwm-dacs which measure the pwm and output a voltage without dips\n\nyes the 0.5 amp one should be enough\n\nR-785.0-0.5\n
Text Notes 5000 6950 0    50   ~ 0
PWM single pole RC filters\n10K + 15nF for 1KHz cutoff\n-43dB down at 146KHz (PWM freq)
$EndSCHEMATC
