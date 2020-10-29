EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 8
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
L OpAmps:TL074 U?
U 1 1 5F8A669A
P 4100 2200
AR Path="/5CEF66AD/5F9DE87B/5F8A669A" Ref="U?"  Part="4" 
AR Path="/5CEF66AD/5F8906B5/5F8A669A" Ref="U19"  Part="1" 
F 0 "U19" H 4250 2050 50  0000 C CNN
F 1 "TL074" H 4200 1950 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 4050 2300 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 4150 2400 50  0001 C CNN
	1    4100 2200
	1    0    0    1   
$EndComp
$Comp
L OpAmps:TL074 U?
U 2 1 5F8A66A0
P 4100 2950
AR Path="/5CEF66AD/5F9DE87B/5F8A66A0" Ref="U?"  Part="3" 
AR Path="/5CEF66AD/5F8906B5/5F8A66A0" Ref="U19"  Part="2" 
F 0 "U19" H 4300 2850 50  0000 C CNN
F 1 "TL074" H 4250 2750 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 4050 3050 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 4150 3150 50  0001 C CNN
	2    4100 2950
	1    0    0    1   
$EndComp
$Comp
L OpAmps:TL074 U?
U 4 1 5F8A66A6
P 5150 2300
AR Path="/5CEF66AD/5F9DE87B/5F8A66A6" Ref="U?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66A6" Ref="U19"  Part="4" 
F 0 "U19" H 5250 2150 50  0000 C CNN
F 1 "TL074" H 5250 2050 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 5100 2400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 5200 2500 50  0001 C CNN
	4    5150 2300
	1    0    0    1   
$EndComp
$Comp
L OpAmps:TL074 U?
U 3 1 5F8A66AC
P 5150 3050
AR Path="/5CEF66AD/5F9DE87B/5F8A66AC" Ref="U?"  Part="2" 
AR Path="/5CEF66AD/5F8906B5/5F8A66AC" Ref="U19"  Part="3" 
F 0 "U19" H 5350 2950 50  0000 C CNN
F 1 "TL074" H 5300 2850 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 5100 3150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 5200 3250 50  0001 C CNN
	3    5150 3050
	1    0    0    1   
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8A66B2
P 3450 2350
AR Path="/5CEF66AD/5F9DE87B/5F8A66B2" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66B2" Ref="R106"  Part="1" 
F 0 "R106" V 3245 2350 50  0000 C CNN
F 1 "30K" V 3336 2350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3450 2350 50  0001 C CNN
F 3 "~" H 3450 2350 50  0001 C CNN
	1    3450 2350
	0    1    1    0   
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8A66B8
P 3450 2450
AR Path="/5CEF66AD/5F9DE87B/5F8A66B8" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66B8" Ref="R107"  Part="1" 
F 0 "R107" V 3650 2450 50  0000 C CNN
F 1 "30K" V 3550 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3450 2450 50  0001 C CNN
F 3 "~" H 3450 2450 50  0001 C CNN
	1    3450 2450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8A66BE
P 3800 2300
AR Path="/5F7954A8/5F8A66BE" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8A66BE" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9DE87B/5F8A66BE" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66BE" Ref="#PWR0200"  Part="1" 
F 0 "#PWR0200" H 3800 2050 50  0001 C CNN
F 1 "GND" H 3805 2127 50  0000 C CNN
F 2 "" H 3800 2300 50  0001 C CNN
F 3 "" H 3800 2300 50  0001 C CNN
	1    3800 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8A66C4
P 3800 3050
AR Path="/5F7954A8/5F8A66C4" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8A66C4" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9DE87B/5F8A66C4" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66C4" Ref="#PWR0201"  Part="1" 
F 0 "#PWR0201" H 3800 2800 50  0001 C CNN
F 1 "GND" H 3805 2877 50  0000 C CNN
F 2 "" H 3800 3050 50  0001 C CNN
F 3 "" H 3800 3050 50  0001 C CNN
	1    3800 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8A66CA
P 4850 3150
AR Path="/5F7954A8/5F8A66CA" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8A66CA" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9DE87B/5F8A66CA" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66CA" Ref="#PWR0202"  Part="1" 
F 0 "#PWR0202" H 4850 2900 50  0001 C CNN
F 1 "GND" H 4855 2977 50  0000 C CNN
F 2 "" H 4850 3150 50  0001 C CNN
F 3 "" H 4850 3150 50  0001 C CNN
	1    4850 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8A66D0
P 4850 2400
AR Path="/5F7954A8/5F8A66D0" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8A66D0" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9DE87B/5F8A66D0" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66D0" Ref="#PWR0203"  Part="1" 
F 0 "#PWR0203" H 4850 2150 50  0001 C CNN
F 1 "GND" H 4855 2227 50  0000 C CNN
F 2 "" H 4850 2400 50  0001 C CNN
F 3 "" H 4850 2400 50  0001 C CNN
	1    4850 2400
	1    0    0    -1  
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8A66D6
P 4000 1850
AR Path="/5CEF66AD/5F9DE87B/5F8A66D6" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66D6" Ref="R108"  Part="1" 
F 0 "R108" V 3795 1850 50  0000 C CNN
F 1 "10K" V 3886 1850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4000 1850 50  0001 C CNN
F 3 "~" H 4000 1850 50  0001 C CNN
	1    4000 1850
	0    1    1    0   
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8A66DC
P 4650 2200
AR Path="/5CEF66AD/5F9DE87B/5F8A66DC" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66DC" Ref="R110"  Part="1" 
F 0 "R110" V 4445 2200 50  0000 C CNN
F 1 "10K" V 4536 2200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4650 2200 50  0001 C CNN
F 3 "~" H 4650 2200 50  0001 C CNN
	1    4650 2200
	0    1    1    0   
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8A66E2
P 5100 1900
AR Path="/5CEF66AD/5F9DE87B/5F8A66E2" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66E2" Ref="R112"  Part="1" 
F 0 "R112" V 4895 1900 50  0000 C CNN
F 1 "10K" V 4986 1900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5100 1900 50  0001 C CNN
F 3 "~" H 5100 1900 50  0001 C CNN
	1    5100 1900
	0    1    1    0   
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8A66E8
P 4100 2600
AR Path="/5CEF66AD/5F9DE87B/5F8A66E8" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66E8" Ref="R109"  Part="1" 
F 0 "R109" V 3895 2600 50  0000 C CNN
F 1 "10K" V 3986 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4100 2600 50  0001 C CNN
F 3 "~" H 4100 2600 50  0001 C CNN
	1    4100 2600
	0    1    1    0   
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8A66EE
P 4650 2950
AR Path="/5CEF66AD/5F9DE87B/5F8A66EE" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66EE" Ref="R111"  Part="1" 
F 0 "R111" V 4445 2950 50  0000 C CNN
F 1 "10K" V 4536 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4650 2950 50  0001 C CNN
F 3 "~" H 4650 2950 50  0001 C CNN
	1    4650 2950
	0    1    1    0   
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8A66F4
P 5150 2750
AR Path="/5CEF66AD/5F9DE87B/5F8A66F4" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8A66F4" Ref="R113"  Part="1" 
F 0 "R113" V 4945 2750 50  0000 C CNN
F 1 "10K" V 5036 2750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5150 2750 50  0001 C CNN
F 3 "~" H 5150 2750 50  0001 C CNN
	1    5150 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	3550 2350 3700 2350
Wire Wire Line
	3700 2350 3700 2100
Wire Wire Line
	3700 2100 3800 2100
Wire Wire Line
	3700 2100 3700 1850
Wire Wire Line
	3700 1850 3900 1850
Connection ~ 3700 2100
Wire Wire Line
	4400 2200 4450 2200
Wire Wire Line
	4750 2200 4800 2200
Wire Wire Line
	4100 1850 4450 1850
Wire Wire Line
	4450 1850 4450 2200
Connection ~ 4450 2200
Wire Wire Line
	4450 2200 4550 2200
Wire Wire Line
	5000 1900 4800 1900
Wire Wire Line
	4800 1900 4800 2200
Connection ~ 4800 2200
Wire Wire Line
	4800 2200 4850 2200
Wire Wire Line
	3550 2450 3700 2450
Wire Wire Line
	3700 2450 3700 2600
Wire Wire Line
	3700 2850 3800 2850
Wire Wire Line
	4000 2600 3700 2600
Connection ~ 3700 2600
Wire Wire Line
	3700 2600 3700 2850
Wire Wire Line
	4400 2950 4450 2950
Wire Wire Line
	4750 2950 4800 2950
Wire Wire Line
	4200 2600 4450 2600
Wire Wire Line
	4450 2600 4450 2950
Connection ~ 4450 2950
Wire Wire Line
	4450 2950 4550 2950
Wire Wire Line
	5050 2750 4800 2750
Wire Wire Line
	4800 2750 4800 2950
Connection ~ 4800 2950
Wire Wire Line
	4800 2950 4850 2950
Wire Wire Line
	5200 1900 5850 1900
Wire Wire Line
	5850 1900 5850 2300
Wire Wire Line
	5250 2750 5800 2750
Wire Wire Line
	5800 2750 5800 3050
Wire Wire Line
	5900 2300 5850 2300
Connection ~ 5850 2300
Wire Wire Line
	5900 3050 5800 3050
Connection ~ 5800 3050
$Comp
L device:R_Small_US R?
U 1 1 5F8EA0AD
P 6000 3050
AR Path="/5CEF66AD/5F9DE87B/5F8EA0AD" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8909DB/5F8EA0AD" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8EA0AD" Ref="R115"  Part="1" 
F 0 "R115" V 5795 3050 50  0000 C CNN
F 1 "10K" V 5886 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6000 3050 50  0001 C CNN
F 3 "~" H 6000 3050 50  0001 C CNN
	1    6000 3050
	0    1    1    0   
$EndComp
$Comp
L device:R_Small_US R?
U 1 1 5F8EA0B3
P 6000 2300
AR Path="/5CEF66AD/5F9DE87B/5F8EA0B3" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8909DB/5F8EA0B3" Ref="R?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8EA0B3" Ref="R114"  Part="1" 
F 0 "R114" V 5795 2300 50  0000 C CNN
F 1 "10K" V 5886 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6000 2300 50  0001 C CNN
F 3 "~" H 6000 2300 50  0001 C CNN
	1    6000 2300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8EA0D1
P 6250 2650
AR Path="/5F7954A8/5F8EA0D1" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8EA0D1" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9DE87B/5F8EA0D1" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8909DB/5F8EA0D1" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8EA0D1" Ref="#PWR0204"  Part="1" 
F 0 "#PWR0204" H 6250 2400 50  0001 C CNN
F 1 "GND" H 6255 2477 50  0000 C CNN
F 2 "" H 6250 2650 50  0001 C CNN
F 3 "" H 6250 2650 50  0001 C CNN
	1    6250 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8EA0D7
P 6700 3400
AR Path="/5F7954A8/5F8EA0D7" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8EA0D7" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9DE87B/5F8EA0D7" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8909DB/5F8EA0D7" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8EA0D7" Ref="#PWR0205"  Part="1" 
F 0 "#PWR0205" H 6700 3150 50  0001 C CNN
F 1 "GND" H 6705 3227 50  0000 C CNN
F 2 "" H 6700 3400 50  0001 C CNN
F 3 "" H 6700 3400 50  0001 C CNN
	1    6700 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2300 6250 2300
Wire Wire Line
	6250 2300 6250 2200
Wire Wire Line
	6250 2350 6250 2300
Connection ~ 6250 2300
Wire Wire Line
	6100 3050 6700 3050
Wire Wire Line
	6700 3050 6700 2950
Wire Wire Line
	6700 3100 6700 3050
Connection ~ 6700 3050
$Comp
L power:+3.3V #PWR?
U 1 1 5F8EA0E5
P 6250 1900
AR Path="/5CEF66AD/5F8909DB/5F8EA0E5" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8EA0E5" Ref="#PWR0206"  Part="1" 
F 0 "#PWR0206" H 6250 1750 50  0001 C CNN
F 1 "+3.3V" H 6265 2073 50  0000 C CNN
F 2 "" H 6250 1900 50  0001 C CNN
F 3 "" H 6250 1900 50  0001 C CNN
	1    6250 1900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5F8EA0EB
P 6700 2650
AR Path="/5CEF66AD/5F8909DB/5F8EA0EB" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8EA0EB" Ref="#PWR0207"  Part="1" 
F 0 "#PWR0207" H 6700 2500 50  0001 C CNN
F 1 "+3.3V" H 6715 2823 50  0000 C CNN
F 2 "" H 6700 2650 50  0001 C CNN
F 3 "" H 6700 2650 50  0001 C CNN
	1    6700 2650
	1    0    0    -1  
$EndComp
Text HLabel 6450 2300 2    50   Output ~ 0
CVC_out
Text HLabel 6950 3050 2    50   Output ~ 0
CVD_out
Wire Wire Line
	6250 2300 6450 2300
Wire Wire Line
	6700 3050 6950 3050
Wire Wire Line
	3350 2350 3100 2350
Wire Wire Line
	3350 2450 3100 2450
Text HLabel 3100 2350 0    50   Input ~ 0
CVC_in
Text HLabel 3100 2450 0    50   Input ~ 0
CVD_in
$Comp
L OpAmps:TL074 U?
U 5 1 5F8EEFFF
P 4350 4200
AR Path="/5CEF66AD/5F9DE87B/5F8EEFFF" Ref="U?"  Part="3" 
AR Path="/5CEF66AD/5F8906B5/5F8EEFFF" Ref="U19"  Part="5" 
F 0 "U19" H 4550 4100 50  0000 C CNN
F 1 "TL074" H 4500 4000 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 4300 4300 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 4400 4400 50  0001 C CNN
	5    4350 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2300 5850 2300
Wire Wire Line
	5450 3050 5800 3050
$Comp
L power:+12V #PWR0208
U 1 1 5F8F3DD6
P 4250 3800
F 0 "#PWR0208" H 4250 3650 50  0001 C CNN
F 1 "+12V" H 4265 3973 50  0000 C CNN
F 2 "" H 4250 3800 50  0001 C CNN
F 3 "" H 4250 3800 50  0001 C CNN
	1    4250 3800
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR0209
U 1 1 5F8F5662
P 4250 4600
F 0 "#PWR0209" H 4250 4700 50  0001 C CNN
F 1 "-12V" H 4265 4773 50  0000 C CNN
F 2 "" H 4250 4600 50  0001 C CNN
F 3 "" H 4250 4600 50  0001 C CNN
	1    4250 4600
	-1   0    0    1   
$EndComp
$Comp
L device:C_Small C29
U 1 1 5F8F5F99
P 4000 3950
F 0 "C29" H 4092 3996 50  0000 L CNN
F 1 "100nF" H 4092 3905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4000 3950 50  0001 C CNN
F 3 "~" H 4000 3950 50  0001 C CNN
	1    4000 3950
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C30
U 1 1 5F8F69F9
P 4000 4650
F 0 "C30" H 4092 4696 50  0000 L CNN
F 1 "100nF" H 4092 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4000 4650 50  0001 C CNN
F 3 "~" H 4000 4650 50  0001 C CNN
	1    4000 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3800 4250 3850
Wire Wire Line
	4000 3850 4250 3850
Connection ~ 4250 3850
Wire Wire Line
	4250 3850 4250 3900
Wire Wire Line
	4250 4500 4250 4550
Wire Wire Line
	4000 4550 4250 4550
Connection ~ 4250 4550
Wire Wire Line
	4250 4550 4250 4600
$Comp
L power:GND #PWR?
U 1 1 5F8FA476
P 4000 4050
AR Path="/5F7954A8/5F8FA476" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8FA476" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9DE87B/5F8FA476" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8FA476" Ref="#PWR0210"  Part="1" 
F 0 "#PWR0210" H 4000 3800 50  0001 C CNN
F 1 "GND" H 4005 3877 50  0000 C CNN
F 2 "" H 4000 4050 50  0001 C CNN
F 3 "" H 4000 4050 50  0001 C CNN
	1    4000 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8FA963
P 4000 4750
AR Path="/5F7954A8/5F8FA963" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8FA963" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F9DE87B/5F8FA963" Ref="#PWR?"  Part="1" 
AR Path="/5CEF66AD/5F8906B5/5F8FA963" Ref="#PWR0211"  Part="1" 
F 0 "#PWR0211" H 4000 4500 50  0001 C CNN
F 1 "GND" H 4005 4577 50  0000 C CNN
F 2 "" H 4000 4750 50  0001 C CNN
F 3 "" H 4000 4750 50  0001 C CNN
	1    4000 4750
	1    0    0    -1  
$EndComp
$Comp
L device:D_Schottky D16
U 1 1 5F98E5B2
P 6250 2050
F 0 "D16" V 6204 2129 50  0000 L CNN
F 1 "D_Schottky" V 6295 2129 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6250 2050 50  0001 C CNN
F 3 "~" H 6250 2050 50  0001 C CNN
	1    6250 2050
	0    1    1    0   
$EndComp
$Comp
L device:D_Schottky D17
U 1 1 5F98EFD4
P 6250 2500
F 0 "D17" V 6204 2579 50  0000 L CNN
F 1 "D_Schottky" V 6295 2579 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6250 2500 50  0001 C CNN
F 3 "~" H 6250 2500 50  0001 C CNN
	1    6250 2500
	0    1    1    0   
$EndComp
$Comp
L device:D_Schottky D18
U 1 1 5F98F4AB
P 6700 2800
F 0 "D18" V 6654 2879 50  0000 L CNN
F 1 "D_Schottky" V 6745 2879 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6700 2800 50  0001 C CNN
F 3 "~" H 6700 2800 50  0001 C CNN
	1    6700 2800
	0    1    1    0   
$EndComp
$Comp
L device:D_Schottky D19
U 1 1 5F98FB31
P 6700 3250
F 0 "D19" V 6654 3329 50  0000 L CNN
F 1 "D_Schottky" V 6745 3329 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6700 3250 50  0001 C CNN
F 3 "~" H 6700 3250 50  0001 C CNN
	1    6700 3250
	0    1    1    0   
$EndComp
$EndSCHEMATC