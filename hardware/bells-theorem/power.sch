EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 8
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
L device:CP1 C4
U 1 1 574FF496
P 4450 3100
F 0 "C4" H 4475 3200 50  0000 L CNN
F 1 "10uF" H 4475 3000 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 4450 3100 50  0001 C CNN
F 3 "" H 4450 3100 50  0000 C CNN
	1    4450 3100
	1    0    0    -1  
$EndComp
$Comp
L device:CP1 C5
U 1 1 574FF4E0
P 4450 3400
F 0 "C5" H 4475 3500 50  0000 L CNN
F 1 "10uF" H 4475 3300 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 4450 3400 50  0001 C CNN
F 3 "" H 4450 3400 50  0000 C CNN
	1    4450 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 58736C52
P 3650 3250
F 0 "#PWR04" H 3650 3000 50  0001 C CNN
F 1 "GND" H 3650 3100 50  0000 C CNN
F 2 "" H 3650 3250 50  0000 C CNN
F 3 "" H 3650 3250 50  0000 C CNN
	1    3650 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 58736C79
P 4750 3250
F 0 "#PWR010" H 4750 3000 50  0001 C CNN
F 1 "GND" H 4750 3100 50  0000 C CNN
F 2 "" H 4750 3250 50  0000 C CNN
F 3 "" H 4750 3250 50  0000 C CNN
	1    4750 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR07
U 1 1 58736CBC
P 4450 2900
F 0 "#PWR07" H 4450 2750 50  0001 C CNN
F 1 "+12V" H 4450 3040 50  0000 C CNN
F 2 "" H 4450 2900 50  0000 C CNN
F 3 "" H 4450 2900 50  0000 C CNN
	1    4450 2900
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5874D051
P 4750 3600
F 0 "#FLG02" H 4750 3695 50  0001 C CNN
F 1 "PWR_FLAG" H 4800 3550 50  0000 C CNN
F 2 "" H 4750 3600 50  0000 C CNN
F 3 "" H 4750 3600 50  0000 C CNN
	1    4750 3600
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR08
U 1 1 5874D505
P 4450 3600
F 0 "#PWR08" H 4450 3700 50  0001 C CNN
F 1 "-12V" H 4450 3750 50  0000 C CNN
F 2 "" H 4450 3600 50  0000 C CNN
F 3 "" H 4450 3600 50  0000 C CNN
	1    4450 3600
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 587507A3
P 4750 2900
F 0 "#FLG01" H 4750 2995 50  0001 C CNN
F 1 "PWR_FLAG" H 4750 3080 50  0000 C CNN
F 2 "" H 4750 2900 50  0000 C CNN
F 3 "" H 4750 2900 50  0000 C CNN
	1    4750 2900
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5B65E029
P 4750 3250
F 0 "#FLG03" H 4750 3345 50  0001 C CNN
F 1 "PWR_FLAG" H 4750 3430 50  0000 C CNN
F 2 "" H 4750 3250 50  0000 C CNN
F 3 "" H 4750 3250 50  0000 C CNN
	1    4750 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2900 4450 2950
Connection ~ 4450 2900
Connection ~ 4450 3250
Wire Wire Line
	4450 3250 4750 3250
Connection ~ 4750 3250
Wire Wire Line
	4450 3550 4450 3600
Connection ~ 4450 3600
Wire Wire Line
	4450 2900 4750 2900
Wire Wire Line
	4750 3600 4450 3600
Wire Wire Line
	3500 3250 3650 3250
Wire Wire Line
	3500 3250 3500 3350
Connection ~ 3500 3250
Wire Wire Line
	3500 3150 3500 3250
Wire Wire Line
	3500 2900 3500 3050
Wire Wire Line
	3000 2900 3500 2900
Wire Wire Line
	3000 3050 3000 2900
Wire Wire Line
	3500 3600 3500 3450
Wire Wire Line
	3000 3600 3500 3600
Wire Wire Line
	3000 3450 3000 3600
Wire Wire Line
	2850 3250 3000 3250
Wire Wire Line
	3000 3150 3000 3250
Wire Wire Line
	3000 3350 3000 3250
Connection ~ 3000 3250
$Comp
L power:GND #PWR01
U 1 1 58749A7A
P 2850 3250
F 0 "#PWR01" H 2850 3000 50  0001 C CNN
F 1 "GND" H 2850 3100 50  0000 C CNN
F 2 "" H 2850 3250 50  0000 C CNN
F 3 "" H 2850 3250 50  0000 C CNN
	1    2850 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3600 4450 3600
Wire Wire Line
	4250 2900 4450 2900
Wire Wire Line
	3650 3250 4450 3250
Connection ~ 3650 3250
$Comp
L device:D_Schottky D1
U 1 1 5C9BC631
P 4100 2900
F 0 "D1" H 4100 2684 50  0000 C CNN
F 1 "D_Schottky" H 4100 2775 50  0000 C CNN
F 2 "Diode_SMD:D_SMA-SMB_Universal_Handsoldering" H 4100 2900 50  0001 C CNN
F 3 "" H 4100 2900 50  0001 C CNN
	1    4100 2900
	-1   0    0    1   
$EndComp
$Comp
L device:D_Schottky D2
U 1 1 5C9BC6CC
P 4100 3600
F 0 "D2" H 4100 3816 50  0000 C CNN
F 1 "D_Schottky" H 4100 3725 50  0000 C CNN
F 2 "Diode_SMD:D_SMA-SMB_Universal_Handsoldering" H 4100 3600 50  0001 C CNN
F 3 "" H 4100 3600 50  0001 C CNN
	1    4100 3600
	1    0    0    -1  
$EndComp
$Comp
L device:Ferrite_Bead_Small L1
U 1 1 5C9BC8AC
P 3700 2900
F 0 "L1" V 3463 2900 50  0000 C CNN
F 1 "Ferrite_Bead_Small" V 3554 2900 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3630 2900 50  0001 C CNN
F 3 "" H 3700 2900 50  0001 C CNN
	1    3700 2900
	0    1    1    0   
$EndComp
$Comp
L device:Ferrite_Bead_Small L2
U 1 1 5C9BC93D
P 3700 3600
F 0 "L2" V 3800 3600 50  0000 C CNN
F 1 "Ferrite_Bead_Small" V 3900 3600 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3630 3600 50  0001 C CNN
F 3 "" H 3700 3600 50  0001 C CNN
	1    3700 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 3600 3600 3600
Connection ~ 3500 3600
Wire Wire Line
	3800 3600 3950 3600
Wire Wire Line
	3500 2900 3600 2900
Connection ~ 3500 2900
Wire Wire Line
	3800 2900 3950 2900
$Comp
L Connectors_Generic:Conn_02x05_Odd_Even J1
U 1 1 5CEFB9A1
P 3300 3250
F 0 "J1" H 3350 2825 50  0000 C CNN
F 1 "Conn_02x05_Odd_Even" H 3700 3700 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x05_P2.54mm_Vertical" H 3300 3250 50  0001 C CNN
F 3 "~" H 3300 3250 50  0001 C CNN
	1    3300 3250
	-1   0    0    1   
$EndComp
$Comp
L regulator_linear:L7805 U2
U 1 1 5F8CF7D5
P 3500 4450
F 0 "U2" H 3500 4692 50  0000 C CNN
F 1 "L7805" H 3500 4601 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline_Wide" H 3525 4300 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 3500 4400 50  0001 C CNN
	1    3500 4450
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C1
U 1 1 5F8D08CF
P 3050 4650
F 0 "C1" H 3060 4720 50  0000 L CNN
F 1 "100nF" H 3060 4570 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3050 4650 50  0001 C CNN
F 3 "" H 3050 4650 50  0000 C CNN
	1    3050 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR085
U 1 1 5F8D145B
P 3500 4750
F 0 "#PWR085" H 3500 4500 50  0001 C CNN
F 1 "GND" H 3500 4600 50  0000 C CNN
F 2 "" H 3500 4750 50  0000 C CNN
F 3 "" H 3500 4750 50  0000 C CNN
	1    3500 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4450 3050 4450
Wire Wire Line
	3050 4450 3050 4550
$Comp
L power:GND #PWR084
U 1 1 5F8D2C9D
P 3050 4750
F 0 "#PWR084" H 3050 4500 50  0001 C CNN
F 1 "GND" H 3050 4600 50  0000 C CNN
F 2 "" H 3050 4750 50  0000 C CNN
F 3 "" H 3050 4750 50  0000 C CNN
	1    3050 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR087
U 1 1 5F8D30DE
P 3900 4850
F 0 "#PWR087" H 3900 4600 50  0001 C CNN
F 1 "GND" H 3900 4700 50  0000 C CNN
F 2 "" H 3900 4850 50  0000 C CNN
F 3 "" H 3900 4850 50  0000 C CNN
	1    3900 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 4450 3900 4450
Wire Wire Line
	3900 4450 3900 4550
$Comp
L power:+12V #PWR083
U 1 1 5F8D46DF
P 3050 4450
F 0 "#PWR083" H 3050 4300 50  0001 C CNN
F 1 "+12V" H 3050 4590 50  0000 C CNN
F 2 "" H 3050 4450 50  0000 C CNN
F 3 "" H 3050 4450 50  0000 C CNN
	1    3050 4450
	1    0    0    -1  
$EndComp
Connection ~ 3050 4450
$Comp
L power:+5V #PWR086
U 1 1 5F8D529B
P 3900 4450
F 0 "#PWR086" H 3900 4300 50  0001 C CNN
F 1 "+5V" H 3915 4623 50  0000 C CNN
F 2 "" H 3900 4450 50  0001 C CNN
F 3 "" H 3900 4450 50  0001 C CNN
	1    3900 4450
	1    0    0    -1  
$EndComp
Connection ~ 3900 4450
$Comp
L device:CP1 C2
U 1 1 5F8D6113
P 3900 4700
F 0 "C2" H 3925 4800 50  0000 L CNN
F 1 "1uF" H 3925 4600 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 3900 4700 50  0001 C CNN
F 3 "" H 3900 4700 50  0000 C CNN
	1    3900 4700
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C3
U 1 1 5F96CE64
P 4350 4650
F 0 "C3" H 4360 4720 50  0000 L CNN
F 1 "100nF" H 4360 4570 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4350 4650 50  0001 C CNN
F 3 "" H 4350 4650 50  0000 C CNN
	1    4350 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5F96CE6A
P 4800 4750
F 0 "#PWR0101" H 4800 4500 50  0001 C CNN
F 1 "GND" H 4800 4600 50  0000 C CNN
F 2 "" H 4800 4750 50  0000 C CNN
F 3 "" H 4800 4750 50  0000 C CNN
	1    4800 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4450 4350 4550
$Comp
L power:GND #PWR0102
U 1 1 5F96CE72
P 4350 4750
F 0 "#PWR0102" H 4350 4500 50  0001 C CNN
F 1 "GND" H 4350 4600 50  0000 C CNN
F 2 "" H 4350 4750 50  0000 C CNN
F 3 "" H 4350 4750 50  0000 C CNN
	1    4350 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5F96CE78
P 5200 4850
F 0 "#PWR0103" H 5200 4600 50  0001 C CNN
F 1 "GND" H 5200 4700 50  0000 C CNN
F 2 "" H 5200 4850 50  0000 C CNN
F 3 "" H 5200 4850 50  0000 C CNN
	1    5200 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4450 5200 4450
Wire Wire Line
	5200 4450 5200 4550
$Comp
L device:CP1 C6
U 1 1 5F96CE8E
P 5200 4700
F 0 "C6" H 5225 4800 50  0000 L CNN
F 1 "1uF" H 5225 4600 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 5200 4700 50  0001 C CNN
F 3 "" H 5200 4700 50  0000 C CNN
	1    5200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4450 3900 4450
$Comp
L regulator_linear:L78L33_TO92 U3
U 1 1 5F96FA82
P 4800 4450
F 0 "U3" H 4800 4692 50  0000 C CNN
F 1 "L78L33_TO92" H 4800 4601 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline_Wide" H 4800 4675 50  0001 C CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/15/55/e5/aa/23/5b/43/fd/CD00000446.pdf/files/CD00000446.pdf/jcr:content/translations/en.CD00000446.pdf" H 4800 4400 50  0001 C CNN
	1    4800 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 4450 4350 4450
Connection ~ 4350 4450
$Comp
L power:+3.3V #PWR0104
U 1 1 5F97305A
P 5200 4450
F 0 "#PWR0104" H 5200 4300 50  0001 C CNN
F 1 "+3.3V" H 5215 4623 50  0000 C CNN
F 2 "" H 5200 4450 50  0001 C CNN
F 3 "" H 5200 4450 50  0001 C CNN
	1    5200 4450
	1    0    0    -1  
$EndComp
Connection ~ 5200 4450
$EndSCHEMATC
