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
Wire Wire Line
	1900 2800 4600 2800
Wire Wire Line
	1900 2700 4600 2700
Wire Wire Line
	3550 5850 1250 5850
Wire Wire Line
	3650 5750 1250 5750
Wire Wire Line
	3750 5650 1250 5650
Wire Wire Line
	3850 5550 1250 5550
Wire Wire Line
	3950 5450 1250 5450
Wire Wire Line
	4050 5350 1250 5350
Wire Wire Line
	4150 5250 1250 5250
Wire Wire Line
	4600 1600 2000 1600
Wire Wire Line
	4600 1500 2100 1500
Wire Wire Line
	4600 1400 2200 1400
Wire Wire Line
	4600 1200 2400 1200
Wire Wire Line
	3550 5850 3550 6550
Connection ~ 3550 5850
Wire Wire Line
	3550 4850 3550 5850
Wire Wire Line
	3350 4850 3550 4850
Connection ~ 3650 5750
Wire Wire Line
	3650 4750 3650 5750
Wire Wire Line
	3350 4750 3650 4750
Connection ~ 3750 5650
Wire Wire Line
	3750 4650 3750 5650
Wire Wire Line
	3350 4650 3750 4650
Connection ~ 3850 5550
Wire Wire Line
	3850 4550 3850 5550
Wire Wire Line
	3350 4550 3850 4550
Connection ~ 3950 5450
Wire Wire Line
	3950 4450 3950 5450
Wire Wire Line
	3350 4450 3950 4450
Connection ~ 4050 5350
Wire Wire Line
	4050 4350 4050 5350
Wire Wire Line
	3350 4350 4050 4350
Connection ~ 4150 5250
Wire Wire Line
	4150 4250 4150 5250
Wire Wire Line
	3350 4250 4150 4250
Wire Wire Line
	4150 3850 3350 3850
Wire Wire Line
	4150 2900 4150 3850
Wire Wire Line
	4600 2900 4150 2900
Wire Wire Line
	4050 3750 3350 3750
Wire Wire Line
	4050 2600 4050 3750
Wire Wire Line
	4600 2600 4050 2600
Wire Wire Line
	3950 2500 4600 2500
Wire Wire Line
	3950 3650 3950 2500
Wire Wire Line
	3350 3650 3950 3650
Wire Wire Line
	3850 3550 3350 3550
Wire Wire Line
	3850 2000 3850 3550
Wire Wire Line
	4600 2000 3850 2000
Wire Wire Line
	3750 1900 4600 1900
Wire Wire Line
	3750 3450 3750 1900
Wire Wire Line
	3350 3450 3750 3450
Wire Wire Line
	3650 3350 3350 3350
Wire Wire Line
	3650 1800 3650 3350
Wire Wire Line
	4600 1800 3650 1800
Wire Wire Line
	3550 1700 4600 1700
Wire Wire Line
	3550 3250 3550 1700
Wire Wire Line
	3350 3250 3550 3250
NoConn ~ 4600 2400
NoConn ~ 4600 2300
NoConn ~ 4600 2200
NoConn ~ 4600 2100
$Sheet
S 2700 3150 650  1800
U 5FDB1881
F0 "right outs" 50
F1 "rightouts.sch" 50
F2 "AB" O R 3350 4350 50 
F3 "BC" O R 3350 4550 50 
F4 "ABC" O R 3350 4450 50 
F5 "BCD" O R 3350 4750 50 
F6 "ABCD" O R 3350 4650 50 
F7 "AB_in" I R 3350 3350 50 
F8 "ABC_in" I R 3350 3450 50 
F9 "BC_in" I R 3350 3550 50 
F10 "BCD_in" I R 3350 3750 50 
F11 "ABCD_in" I R 3350 3650 50 
F12 "C" O R 3350 4850 50 
F13 "B" O R 3350 4250 50 
F14 "B_in" I R 3350 3250 50 
F15 "C_in" I R 3350 3850 50 
$EndSheet
Connection ~ 2000 6150
Wire Wire Line
	2000 1600 2000 6150
Connection ~ 2100 6050
Wire Wire Line
	2100 1500 2100 6050
Connection ~ 2200 5950
Wire Wire Line
	2200 1400 2200 5950
Connection ~ 2400 5050
Wire Wire Line
	2400 1200 2400 5050
Connection ~ 1900 6250
Wire Wire Line
	1900 3550 1900 6250
Wire Wire Line
	750  3550 1900 3550
Wire Wire Line
	750  2700 750  3550
Wire Wire Line
	950  2700 750  2700
Connection ~ 1800 6350
Wire Wire Line
	1800 3450 1800 6350
Wire Wire Line
	850  3450 1800 3450
Wire Wire Line
	850  2800 850  3450
Wire Wire Line
	950  2800 850  2800
Wire Wire Line
	2400 5050 2400 6550
Wire Wire Line
	6800 3000 7900 3000
Wire Wire Line
	6800 2900 7900 2900
Wire Wire Line
	7700 2800 7900 2800
Wire Wire Line
	7700 1400 7700 2800
Wire Wire Line
	6800 1400 7700 1400
Wire Wire Line
	7800 2700 7900 2700
Wire Wire Line
	7800 1300 7800 2700
Wire Wire Line
	6800 1300 7800 1300
Wire Wire Line
	7200 3100 7900 3100
Wire Wire Line
	7200 3300 7200 3100
Wire Wire Line
	6800 3300 7200 3300
Wire Wire Line
	7300 3200 7300 4550
Wire Wire Line
	7300 3200 7900 3200
Text HLabel 9650 1400 1    50   Output ~ 0
A
Text HLabel 8850 1400 1    50   Output ~ 0
D
$Sheet
S 7900 2300 800  1300
U 5FAD26AB
F0 "left outs" 50
F1 "leftouts.sch" 50
F2 "A" O R 8700 2450 50 
F3 "D" O R 8700 3250 50 
F4 "DA" O R 8700 2650 50 
F5 "DAB" O R 8700 2550 50 
F6 "DA_in" I L 7900 2900 50 
F7 "DAB_in" I L 7900 2800 50 
F8 "D_in" I L 7900 3200 50 
F9 "A_in" I L 7900 2700 50 
F10 "CDA" O R 8700 2750 50 
F11 "CD" O R 8700 2850 50 
F12 "CD_in" I L 7900 3100 50 
F13 "CDA_in" I L 7900 3000 50 
$EndSheet
Wire Wire Line
	7400 4300 8350 4300
Wire Wire Line
	7400 2800 7400 4300
Wire Wire Line
	6800 2800 7400 2800
NoConn ~ 6800 3200
NoConn ~ 6800 3100
Wire Wire Line
	7500 4200 8350 4200
Wire Wire Line
	7500 2700 7500 4200
Wire Wire Line
	6800 2700 7500 2700
Wire Wire Line
	7600 4100 8350 4100
Wire Wire Line
	7600 2600 7600 4100
Wire Wire Line
	6800 2600 7600 2600
NoConn ~ 6800 2500
Wire Wire Line
	4500 4550 7300 4550
Wire Wire Line
	4500 3000 4500 4550
Wire Wire Line
	4600 3000 4500 3000
NoConn ~ 6800 2300
NoConn ~ 6800 1500
NoConn ~ 6800 1200
Wire Wire Line
	8950 2200 8950 1400
Wire Wire Line
	8950 2200 6800 2200
Wire Wire Line
	9050 1400 9050 2100
Wire Wire Line
	9050 2100 6800 2100
Wire Wire Line
	9150 2000 9150 1400
Wire Wire Line
	9150 2000 6800 2000
Connection ~ 9750 1900
Wire Wire Line
	9750 1400 9750 1900
Connection ~ 9850 1800
Wire Wire Line
	9850 1400 9850 1800
Connection ~ 9950 1700
Wire Wire Line
	9950 1400 9950 1700
Wire Wire Line
	9950 2150 10550 2150
Wire Wire Line
	9750 1900 6800 1900
Wire Wire Line
	9850 1800 6800 1800
Wire Wire Line
	9950 1700 6800 1700
Wire Wire Line
	9950 2150 9950 1700
NoConn ~ 4600 3100
NoConn ~ 4600 3200
NoConn ~ 4600 3300
$Comp
L teensy:Teensy4.1 U1
U 1 1 5F831BBA
P 5700 3150
F 0 "U1" H 5700 5715 50  0000 C CNN
F 1 "Teensy4.1" H 5700 5624 50  0000 C CNN
F 2 "teensy:Teensy41" H 5300 3550 50  0001 C CNN
F 3 "" H 5300 3550 50  0001 C CNN
	1    5700 3150
	1    0    0    -1  
$EndComp
Text Notes 4850 6700 0    50   ~ 0
PWM single pole RC filters\n10K + 15nF for 1KHz cutoff\n-43dB down at 146KHz (PWM freq)
Text Notes 8000 6550 0    50   ~ 0
notes from Jim Matheson:\non here you can see you can get 12 bit pwm at 36 khz\n\nhttps://www.pjrc.com/teensy/td_pulse.html\nPulse Width and Tone on Teensy with Arduino\npjrc.com\n\nand sending an “analogwrite” to a pin is easier and faster than i2c\n\nor you could get a 8 channel spi dac\n\nyou can save about 40mA by using a recom switching 7805 replacement\n\nRecom: R-78?\n\nthere are even pwm-dacs which measure the pwm and output a voltage without dips\n\nyes the 0.5 amp one should be enough\n\nR-785.0-0.5\n
Connection ~ 10150 4200
Wire Wire Line
	10150 3450 10150 4200
Wire Wire Line
	10550 3450 10150 3450
Connection ~ 10050 4100
Wire Wire Line
	10050 3350 10050 4100
Wire Wire Line
	10550 3350 10050 3350
Wire Wire Line
	10050 4100 10050 4500
Wire Wire Line
	9450 4100 10050 4100
Wire Wire Line
	10150 4200 10150 4500
Wire Wire Line
	9450 4200 10150 4200
Wire Wire Line
	10250 4300 10250 4500
Connection ~ 10250 4300
Wire Wire Line
	9450 4300 10250 4300
$Sheet
S 8350 4000 1100 750 
U 5F8909DB
F0 "scaleleft" 50
F1 "scaleleft.sch" 50
F2 "CVA_in" I R 9450 4200 50 
F3 "CVB_in" I R 9450 4100 50 
F4 "IN_in" I R 9450 4300 50 
F5 "CVA_out" O L 8350 4200 50 
F6 "CVB_out" O L 8350 4100 50 
F7 "IN_out" O L 8350 4300 50 
$EndSheet
$Sheet
S 950  2500 950  750 
U 5F8906B5
F0 "scaleright" 50
F1 "scaleright.sch" 50
F2 "CVC_out" O R 1900 2700 50 
F3 "CVD_out" O R 1900 2800 50 
F4 "CVC_in" I L 950 2700 50 
F5 "CVD_in" I L 950 2800 50 
$EndSheet
Wire Wire Line
	10250 3550 10250 4300
Text HLabel 3750 6550 3    50   Output ~ 0
ABCD
Text HLabel 3850 6550 3    50   Output ~ 0
BC
Wire Wire Line
	10250 3550 10550 3550
Wire Wire Line
	1250 5050 2400 5050
Text HLabel 2500 6550 3    50   Input ~ 0
ENCB_B
Text HLabel 2300 6550 3    50   Input ~ 0
ENCB_SW
Text HLabel 2400 6550 3    50   Input ~ 0
ENCB_A
Text HLabel 1800 6550 3    50   Input ~ 0
CVD
Text HLabel 1900 6550 3    50   Input ~ 0
CVC
Text HLabel 2200 6550 3    50   Input ~ 0
ENCC_SW
Text HLabel 2100 6550 3    50   Input ~ 0
ENCC_B
Text HLabel 2000 6550 3    50   Input ~ 0
ENCC_A
Wire Wire Line
	2200 5950 2200 6550
Wire Wire Line
	2100 6050 2100 6550
Wire Wire Line
	2000 6150 2000 6550
Wire Wire Line
	1900 6250 1900 6550
Wire Wire Line
	1800 6350 1800 6550
Wire Wire Line
	1250 6350 1800 6350
Text HLabel 3550 6550 3    50   Output ~ 0
C
Wire Wire Line
	1250 6250 1900 6250
Wire Wire Line
	1250 6150 2000 6150
Wire Wire Line
	1250 6050 2100 6050
Wire Wire Line
	1250 5950 2200 5950
Wire Wire Line
	3650 6550 3650 5750
Wire Wire Line
	3750 6550 3750 5650
Wire Wire Line
	3850 6550 3850 5550
Wire Wire Line
	3950 6550 3950 5450
Wire Wire Line
	4050 6550 4050 5350
Wire Wire Line
	4150 6550 4150 5250
Wire Wire Line
	4400 1000 4400 3450
Wire Wire Line
	4600 1000 4400 1000
Wire Wire Line
	7000 2400 7000 1100
Connection ~ 7000 2400
Wire Wire Line
	6800 2400 7000 2400
Wire Wire Line
	7000 1100 6800 1100
Wire Wire Line
	7000 3500 7000 2400
$Comp
L power:GND #PWR?
U 1 1 5F95BE5C
P 7000 3500
AR Path="/5F7954A8/5F95BE5C" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F95BE5C" Ref="#PWR0131"  Part="1" 
F 0 "#PWR0131" H 7000 3250 50  0001 C CNN
F 1 "GND" H 7005 3327 50  0000 C CNN
F 2 "" H 7000 3500 50  0001 C CNN
F 3 "" H 7000 3500 50  0001 C CNN
	1    7000 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F95B7C3
P 4400 3450
AR Path="/5F7954A8/5F95B7C3" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F95B7C3" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0130" H 4400 3200 50  0001 C CNN
F 1 "GND" H 4405 3277 50  0000 C CNN
F 2 "" H 4400 3450 50  0001 C CNN
F 3 "" H 4400 3450 50  0001 C CNN
	1    4400 3450
	1    0    0    -1  
$EndComp
NoConn ~ 6000 4450
NoConn ~ 5850 4450
NoConn ~ 5700 4450
NoConn ~ 5550 4450
NoConn ~ 5400 4450
NoConn ~ 4600 4150
NoConn ~ 4600 4050
NoConn ~ 4600 3950
NoConn ~ 4600 3850
NoConn ~ 4600 3750
NoConn ~ 4600 3600
NoConn ~ 4600 3500
NoConn ~ 6800 4050
NoConn ~ 6800 3950
NoConn ~ 6800 3850
NoConn ~ 6800 3750
NoConn ~ 6800 3650
NoConn ~ 6800 3550
Wire Wire Line
	7000 1000 7000 900 
Wire Wire Line
	6800 1000 7000 1000
NoConn ~ 6800 850 
$Comp
L power:+5V #PWR?
U 1 1 5F8CA0BE
P 7000 900
AR Path="/5F7954A8/5F8CA0BE" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8CA0BE" Ref="#PWR088"  Part="1" 
F 0 "#PWR088" H 7000 750 50  0001 C CNN
F 1 "+5V" H 7015 1073 50  0000 C CNN
F 2 "" H 7000 900 50  0001 C CNN
F 3 "" H 7000 900 50  0001 C CNN
	1    7000 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 6550 1250 6550
Wire Wire Line
	1300 6650 1300 6550
Wire Wire Line
	1600 6450 1250 6450
Wire Wire Line
	1600 6400 1600 6450
Wire Wire Line
	1500 4850 1250 4850
Wire Wire Line
	1500 4900 1500 4850
Wire Wire Line
	1300 4750 1250 4750
Wire Wire Line
	1300 4700 1300 4750
$Comp
L power:GND #PWR?
U 1 1 5F866538
P 1500 4900
AR Path="/5F7954A8/5F866538" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F866538" Ref="#PWR055"  Part="1" 
F 0 "#PWR055" H 1500 4650 50  0001 C CNN
F 1 "GND" H 1505 4727 50  0000 C CNN
F 2 "" H 1500 4900 50  0001 C CNN
F 3 "" H 1500 4900 50  0001 C CNN
	1    1500 4900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10500 3750 10550 3750
Wire Wire Line
	10500 3850 10500 3750
Wire Wire Line
	10350 3650 10550 3650
Wire Wire Line
	10350 3600 10350 3650
Wire Wire Line
	10100 2050 10550 2050
Wire Wire Line
	10100 2100 10100 2050
$Comp
L power:GND #PWR?
U 1 1 5F86539D
P 10100 2100
AR Path="/5F7954A8/5F86539D" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F86539D" Ref="#PWR047"  Part="1" 
F 0 "#PWR047" H 10100 1850 50  0001 C CNN
F 1 "GND" H 10105 1927 50  0000 C CNN
F 2 "" H 10100 2100 50  0001 C CNN
F 3 "" H 10100 2100 50  0001 C CNN
	1    10100 2100
	1    0    0    -1  
$EndComp
$Comp
L Connectors_Generic:Conn_01x19 J3
U 1 1 5F85F4C1
P 1050 5650
F 0 "J3" H 1130 5692 50  0000 L CNN
F 1 "right" H 1130 5601 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x19_P2.54mm_Vertical" H 1050 5650 50  0001 C CNN
F 3 "~" H 1050 5650 50  0001 C CNN
	1    1050 5650
	-1   0    0    -1  
$EndComp
$Comp
L Connectors_Generic:Conn_01x19 J2
U 1 1 5F85C3F5
P 10750 2850
F 0 "J2" H 10830 2892 50  0000 L CNN
F 1 "left" H 10830 2801 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x19_P2.54mm_Vertical" H 10750 2850 50  0001 C CNN
F 3 "~" H 10750 2850 50  0001 C CNN
	1    10750 2850
	1    0    0    -1  
$EndComp
Text HLabel 9750 1400 1    50   Input ~ 0
ENCA_A
Text HLabel 9850 1400 1    50   Input ~ 0
ENCA_B
Text HLabel 9950 1400 1    50   Input ~ 0
ENCA_SW
Text HLabel 9550 1400 1    50   Output ~ 0
DAB
Text HLabel 9350 1400 1    50   Output ~ 0
CDA
Text HLabel 3650 6550 3    50   Output ~ 0
BCD
Text HLabel 3950 6550 3    50   Output ~ 0
ABC
Text HLabel 9450 1400 1    50   Output ~ 0
DA
Text HLabel 9250 1400 1    50   Output ~ 0
CD
Text HLabel 4050 6550 3    50   Output ~ 0
AB
Text HLabel 4150 6550 3    50   Output ~ 0
B
Text HLabel 10050 4500 3    50   Input ~ 0
CVB
Text HLabel 10150 4500 3    50   Input ~ 0
CVA
Text HLabel 10250 4500 3    50   Input ~ 0
IN
Text HLabel 8950 1400 1    50   Input ~ 0
ENCD_SW
Text HLabel 9150 1400 1    50   Input ~ 0
ENCD_B
Text HLabel 9050 1400 1    50   Input ~ 0
ENCD_A
$Comp
L power:GND #PWR?
U 1 1 5F97FCF6
P 10500 3850
AR Path="/5F7954A8/5F97FCF6" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F97FCF6" Ref="#PWR06"  Part="1" 
F 0 "#PWR06" H 10500 3600 50  0001 C CNN
F 1 "GND" H 10505 3677 50  0000 C CNN
F 2 "" H 10500 3850 50  0001 C CNN
F 3 "" H 10500 3850 50  0001 C CNN
	1    10500 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F97FCF0
P 1300 6650
AR Path="/5F7954A8/5F97FCF0" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F97FCF0" Ref="#PWR03"  Part="1" 
F 0 "#PWR03" H 1300 6400 50  0001 C CNN
F 1 "GND" H 1305 6477 50  0000 C CNN
F 2 "" H 1300 6650 50  0001 C CNN
F 3 "" H 1300 6650 50  0001 C CNN
	1    1300 6650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8700 2450 9650 2450
Wire Wire Line
	8700 2650 9450 2650
Wire Wire Line
	9650 1400 9650 2250
Wire Wire Line
	9550 1400 9550 2550
Wire Wire Line
	8700 2550 9550 2550
Connection ~ 9550 2550
Wire Wire Line
	9550 2550 10550 2550
Wire Wire Line
	9450 1400 9450 2650
Connection ~ 9450 2650
Wire Wire Line
	9450 2650 10550 2650
Wire Wire Line
	9350 1400 9350 2750
Connection ~ 9150 2000
Wire Wire Line
	9050 2100 9050 3050
Connection ~ 9050 2100
Wire Wire Line
	8950 2200 8950 3150
Connection ~ 8950 2200
Wire Wire Line
	9750 2450 10550 2450
Wire Wire Line
	9750 1900 9750 2450
Wire Wire Line
	9850 2350 10550 2350
Wire Wire Line
	9850 1800 9850 2350
Wire Wire Line
	10550 2250 9650 2250
Connection ~ 9650 2250
Wire Wire Line
	9650 2250 9650 2450
Wire Wire Line
	9350 2750 8700 2750
Wire Wire Line
	9350 2750 10550 2750
Connection ~ 9350 2750
Wire Wire Line
	10550 2850 9250 2850
Wire Wire Line
	10550 2950 9150 2950
Wire Wire Line
	9150 2000 9150 2950
Wire Wire Line
	10550 3050 9050 3050
Wire Wire Line
	10550 3150 8950 3150
Wire Wire Line
	8700 3250 8850 3250
Wire Wire Line
	8850 1400 8850 3250
Connection ~ 8850 3250
Wire Wire Line
	8850 3250 10550 3250
Wire Wire Line
	9250 1400 9250 2850
Connection ~ 9250 2850
Wire Wire Line
	9250 2850 8700 2850
Wire Wire Line
	1250 4950 2500 4950
Connection ~ 2500 4950
Wire Wire Line
	2500 4950 2500 6550
Wire Wire Line
	1250 5150 2300 5150
Connection ~ 2300 5150
Wire Wire Line
	2300 5150 2300 6550
Wire Wire Line
	2500 1100 4600 1100
Wire Wire Line
	2500 1100 2500 4950
Wire Wire Line
	4600 1300 2300 1300
Wire Wire Line
	2300 1300 2300 5150
$Comp
L power:+12V #PWR?
U 1 1 5F97FD02
P 10350 3600
AR Path="/5F7954A8/5F97FD02" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F97FD02" Ref="#PWR05"  Part="1" 
F 0 "#PWR05" H 10350 3450 50  0001 C CNN
F 1 "+12V" H 10365 3773 50  0000 C CNN
F 2 "" H 10350 3600 50  0001 C CNN
F 3 "" H 10350 3600 50  0001 C CNN
	1    10350 3600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR040
U 1 1 60787F95
P 1300 4700
F 0 "#PWR040" H 1300 4550 50  0001 C CNN
F 1 "+3.3V" H 1315 4873 50  0000 C CNN
F 2 "" H 1300 4700 50  0001 C CNN
F 3 "" H 1300 4700 50  0001 C CNN
	1    1300 4700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR039
U 1 1 60788A4D
P 1600 6400
F 0 "#PWR039" H 1600 6250 50  0001 C CNN
F 1 "+5V" H 1615 6573 50  0000 C CNN
F 2 "" H 1600 6400 50  0001 C CNN
F 3 "" H 1600 6400 50  0001 C CNN
	1    1600 6400
	1    0    0    -1  
$EndComp
Text HLabel 10400 1950 0    50   Output ~ 0
np_ctl
Wire Wire Line
	10400 1950 10550 1950
Text HLabel 7150 1600 2    50   Output ~ 0
np_ctl
Wire Wire Line
	6800 1600 7150 1600
$EndSCHEMATC
